#define ZMQ_BUILD_DRAFT_API
#include <future>
#include <chrono>
#include <memory>
#include <vector>
#include <csignal>
#include <zmq.h>

#include "bobik_driver.hpp"
#include "xv11_laser.h"
#include "protocol_types.h"
#include "loguru.hpp"

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <iostream>
#include "uart_transporter.hpp"

#define MSG_START 0xEE

constexpr int BUFFER_SIZE = 1024;
constexpr int TTY_BAUDRATE = 500000;
constexpr int READ_POLL_MS = 100;

#define XV11_PORT_DEFAULT "/dev/ttyUSB0"           // Serial device driver name (sym link to real dev)
#define XV11_BAUD_RATE_DEFAULT 115200              // Serial baud rate

using namespace std::chrono_literals;

bool stop = false;
int serial_port;
struct termios tty;

void *radio;
void *dish;
zmq_msg_t zmq_msg;

BobikDriver::BobikDriver()
{
    future_ = exit_signal_.get_future();
    device = "/dev/ttyUSB0"; // TODO discover
    transporter_ = std::make_unique<UARTTransporter>(device, TTY_BAUDRATE, READ_POLL_MS, BUFFER_SIZE);
    if (transporter_->init() < 0)
    {
        throw std::runtime_error("Failed to initialize transport");
    }

    read_from_arduino_thread_ = std::thread(&BobikDriver::read_from_arduino_thread_func, this, future_);
    read_from_lidar_thread_   = std::thread(&BobikDriver::read_from_lidar_thread_func, this, future_);

    // Setup 0MQ
    void *zmq_ctx = zmq_ctx_new();
    radio = zmq_socket(zmq_ctx, ZMQ_RADIO);
    dish = zmq_socket(zmq_ctx, ZMQ_DISH);
    if (zmq_connect(radio, "udp://192.168.1.2:7655") != 0)
    {
        LOG_F(ERROR, "zmq_connect: %s", zmq_strerror(errno));
        return;
    }
    if (zmq_bind(dish, "udp://*:7654") != 0)
    {
        LOG_F(ERROR, "zmq_bind: %s", zmq_strerror(errno));
        return;
    }
    if (zmq_join(dish, TOPIC_CMD_VEL) != 0)
    {
        LOG_F(ERROR, "Could not subscribe to: %s", TOPIC_CMD_VEL);
        return;
    }

    LOG_F(INFO, "Bobik driver initialized");
}

BobikDriver::~BobikDriver()
{
    LOG_F(INFO, "Bobik driver not yet finished"); // TODO this is not called. See log '[rclcpp]: signal_handler(signal_value=2)'
    stop = true;
    read_from_arduino_thread_.join();
    LOG_F(INFO, "Bobik driver finished"); // TODO this is not called. See log '[rclcpp]: signal_handler(signal_value=2)'
    transporter_->close();
}

void BobikDriver::send_to_zmq_topic(const char *topic, void *data, size_t size) const
{
    if (zmq_msg_init_data(&zmq_msg, data, size, NULL, NULL) == -1)
    {
        LOG_F(ERROR, "Failed to init message for topic %s. %s", topic, zmq_strerror(errno));
        return;
    }
    if (zmq_msg_set_group(&zmq_msg, topic) == -1)
    {
        LOG_F(ERROR, "Failed to set group for topic %s. %s", topic, zmq_strerror(errno));
        return;
    }
    if (zmq_msg_send(&zmq_msg, radio, 0) == -1)
    {
        LOG_F(ERROR, "Failed to send message for topic %s. %s", topic, zmq_strerror(errno));
        return;
    }
    if (zmq_msg_close(&zmq_msg) == -1)
    {
        LOG_F(ERROR, "Failed to send message for topic %s. %s", topic, zmq_strerror(errno));
        return;
    }
}

void BobikDriver::cmd_vel_callback(uint8_t *msg_cmd_vel) const
{
    int written = transporter_->write(DRIVE_CMD, msg_cmd_vel, 6);
    if (written <= 0)
    {
        LOG_F(ERROR, "Failed to write to arduino, %d", written);
    }
    LOG_F(INFO, "msg_cmd_vel written bytes: '%d'", written);
}

/*
 * Reads data from the serial port of XV11 lidar and publishes it as a ROS message.
 */

void BobikDriver::read_from_lidar_thread_func(const std::shared_future<void> &local_future)
{
    std::future_status status;
    #define DEG_VALUES 360
    LaserScan_t ranges;
    ranges.data_type = 0;
    LaserScan_t intensities;
    intensities.data_type = 1;
    uint32_t time_increment;

    // XV11 Lidar
    boost::asio::io_service io;
    xv_11_driver::XV11Laser laser(XV11_PORT_DEFAULT, XV11_BAUD_RATE_DEFAULT, io);

    do
    {
        laser.poll(ranges.data, intensities.data, &time_increment);
        ranges.time_increment = time_increment; //  /1e8
        intensities.time_increment = time_increment; //  /1e8
        send_to_zmq_topic(TOPIC_LIDAR_RANGES, &ranges, sizeof(LaserScan_t));  //  / 1000.0
        send_to_zmq_topic(TOPIC_LIDAR_INTENSITIES, &intensities, sizeof(LaserScan_t));

        status = local_future.wait_for(std::chrono::seconds(0));
    } while (!stop && (status == std::future_status::timeout));
    
}

/*
 * Reads data from the serial port and publishes it as a ROS message.
 */
void BobikDriver::read_from_arduino_thread_func(const std::shared_future<void> &local_future)
{
    std::future_status status;

    // We use a unique_ptr here both to make this a heap allocation and to quiet
    // non-owning pointer warnings from clang-tidy
    std::unique_ptr<uint8_t[]> data_buffer(new uint8_t[BUFFER_SIZE]);
    ssize_t length = 0;

    do
    {
        // Process serial -> ROS 2 data
        if ((length = transporter_->read(data_buffer.get())) >= 0)
        {
            // LOG_F(INFO, "Received bytes ----%d ", length);
            char *bufo = (char *)(data_buffer.get());
            char bufs[1000];
            char *bufc = bufs;
            for (int i = 0; i < length; i++)
            {
                sprintf(bufc, "%02x:", (uint8_t)bufo[i]);
                bufc += 3;
            }
            *bufc = 0;
            // LOG_F(INFO, "Loop read thread--- %d| %s", length, bufs);
            dispatch_from_arduino(data_buffer.get(), length);
        }
        status = local_future.wait_for(std::chrono::seconds(0));
    } while (!stop && (status == std::future_status::timeout));
}

// Distribute messages from arduino to the correct handler based on message type
void BobikDriver::dispatch_from_arduino(uint8_t *data_buffer, ssize_t length)
{
    uint8_t *p = data_buffer;
    do
    {
        if (*p != MSG_START)
        {
            LOG_F(ERROR, "Expected MSG_START but got something else %x", *p);
        }
        p++;

        uint8_t msg_type = *p;
        p++;
        int msg_len = dispatch_msg_from_arduino(msg_type, p);
        p += msg_len;
    } while (p < (data_buffer + length));
}

// Distribute messages from arduino to the correct handler based on message type
int BobikDriver::dispatch_msg_from_arduino(uint8_t msg_type, uint8_t *data_buffer)
{
    if (msg_type == LOOP_TIMESTAMP)
    {
        MsgTimestamp_t *msg = (struct MsgTimestamp_t *)data_buffer;
        LOG_F(MAX, "Time %d", msg->timestamp);
        return sizeof(MsgTimestamp_t);
    }
    if (msg_type == LOOP_END_MSG)
    {
        MsgCRC_t *msg = (struct MsgCRC_t *)data_buffer;
        LOG_F(MAX, "CRC %x", msg->crc);
        return sizeof(MsgCRC_t);
    }
    if (msg_type == LOG4)
    {
        MsgLog4_t *msg = (struct MsgLog4_t *)data_buffer;
        LOG_F(INFO, "LOG %x %x %x %x", msg->log1, msg->log2, msg->log3, msg->log4);
        return sizeof(MsgLog4_t);
    }
    if (msg_type == LOADCELL_UPPER_ARM_LIFT_JOINT)
    {
        MsgLoadCell_t *msg = (struct MsgLoadCell_t *)data_buffer;
        LOG_F(INFO, "Load %d", msg->value);
        return sizeof(MsgLoadCell_t);
    }
    if (msg_type == CASTER_JOINT_STATES)
    {
        send_to_zmq_topic(TOPIC_CASTER_RAW, data_buffer, sizeof(MsgCasterJointStates_t));
        return sizeof(MsgCasterJointStates_t);
    }
    LOG_F(ERROR, "Unknown message type %x", msg_type);
    return 0;
}

// Read data from ZMQ
void BobikDriver::run()
{
    do
    {
        int bytesReceived;
        zmq_msg_t receiveMessage;

        zmq_msg_init(&receiveMessage);
        bytesReceived = zmq_msg_recv(&receiveMessage, dish, 0);
        LOG_F(INFO, "Received bytes: '%d'", bytesReceived);
        if (bytesReceived == -1)
        {
            LOG_F(ERROR, "Failed to receive message from zmq.");
        }
        else
        {
            if (strcmp(zmq_msg_group(&receiveMessage), TOPIC_CMD_VEL) == 0)
            {
                char *h = (char *)&receiveMessage;
                LOG_F(INFO, "CMD_VEL from ros2 topic: %s, data: %02X:%02X:%02X:%02X:%02X:%02X, size: %d\n", zmq_msg_group(&receiveMessage), h[0],h[1],h[2],h[3],h[4],h[5], bytesReceived);
                void *data_buffer = zmq_msg_data(&receiveMessage);
                cmd_vel_callback((uint8_t *)data_buffer);
            }
            else
            {
                LOG_F(INFO, "from ros2 topic: %s, data: %s, size: %d\n", zmq_msg_group(&receiveMessage), (char *)zmq_msg_data(&receiveMessage), bytesReceived);
            }

        }

        zmq_msg_close(&receiveMessage);
    } while (!stop);
}

void sigint_handler(int signum)
{
    stop = true;
}

/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char *argv[])
{
    loguru::init(argc, argv);
    // std::signal(SIGINT, sigint_handler); // works but needs to unblock by receiving a message from ros2
    BobikDriver bobik_driver;
    bobik_driver.run();
    return 0;
}
