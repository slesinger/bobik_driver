#include <future>
#include <chrono>
#include <memory>
#include <vector>
#include <csignal>
#include "loguru.hpp"
#include "bobik_driver.hpp"
#include "bobik_zmq.hpp"
#include "bobik_kinect.hpp"

#include "xv11_laser.h"
#include "protocol_types.h"

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
#include <iostream>

#include "libfreenect.h"

#define MSG_START 0xEE

constexpr int BUFFER_SIZE = 1024;
constexpr int TTY_BAUDRATE = 19200; //500000 is too much for voltage converter
constexpr int READ_POLL_MS = 100;

#define XV11_PORT_DEFAULT "/dev/ttyUSB0"           // Lidar Serial device driver name (sym link to real dev)
#define XV11_BAUD_RATE_DEFAULT 115200              // Serial baud rate

using namespace std::chrono_literals;

bool stop = false;
int serial_port;
struct termios tty;

BobikDriver::BobikDriver()
{

    future_ = exit_signal_.get_future();

    // Setup Arduino serial
    device = "/dev/ttyTHS1"; // Arduino serial
    transporter_ = std::make_unique<UARTTransporter>(device, TTY_BAUDRATE, READ_POLL_MS, BUFFER_SIZE);
    if (transporter_->init() < 0)
    {
        throw std::runtime_error("Failed to initialize transport");
    }

    // Start threads
    read_from_arduino_thread_ = std::thread(&BobikDriver::read_from_arduino_thread_func, this, future_);
    read_from_lidar_thread_   = std::thread(&BobikDriver::read_from_lidar_thread_func, this, future_);
    read_from_kinect_thread_   = std::thread(&BobikDriver::read_from_kinect_thread_func, this, future_);
//    serve_reqresp_thread_     = std::thread(&BobikDriver::serve_reqresp_thread_func, this, future_);

    LOG_F(INFO, "Bobik driver initialized");
}

BobikDriver::~BobikDriver()
{
    LOG_F(INFO, "Bobik driver shutting down");
    stop = true;  // TODO wait for thread join
    this->bobik_kinect.shutdown();
    read_from_arduino_thread_.join();
    transporter_->close();
    LOG_F(INFO, "Bobik driver finished"); // TODO this is not called. See log '[rclcpp]: signal_handler(signal_value=2)'
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
    uint32_t time_increment;

    // XV11 Lidar
    boost::asio::io_service io;
    xv_11_driver::XV11Laser laser(XV11_PORT_DEFAULT, XV11_BAUD_RATE_DEFAULT, io);

    do
    {
        laser.poll(ranges.data, &time_increment);
        ranges.time_increment = time_increment; //  /1e8
        bobik_zmq.send_to_zmq_topic(TOPIC_LIDAR_RANGES, &ranges, sizeof(LaserScan_t));  //  / 1000.0

        status = local_future.wait_for(std::chrono::seconds(0));
    } while (!stop && (status == std::future_status::timeout));

}

void BobikDriver::read_from_kinect_thread_func(const std::shared_future<void> &local_future)
{
    std::future_status status;
    do
    {
        if (int rc = this->bobik_kinect.run() < 0)
        {
            LOG_F(WARNING, "Kinect USB events error %d", rc);
        }
        status = local_future.wait_for(std::chrono::seconds(0));
    } while (!stop && (status == std::future_status::timeout));
}

/*
 * Receive requests from 0MQ, dispach them and send synchronous response back to bobik_robot
 */
/*
void BobikDriver::serve_reqresp_thread_func(const std::shared_future<void> &local_future)
{
    std::future_status status;
    do {
        char *req_str = s_recv(zmq_rep);
        printf ("Received request: [%s]\n", req_str);
        int size = zmq_send(zmq_rep, "World", strlen ("World"), 0);
        free (req_str);
    } while (!stop && (status == std::future_status::timeout));
}
*/
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
    LOG_F(INFO, "Arduino thread started");

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
    // LOG_F(INFO, "msg_type: '%d'", msg_type);
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
        bobik_zmq.send_to_zmq_topic(TOPIC_CASTER_RAW, data_buffer, sizeof(MsgCasterJointStates_t));
        return sizeof(MsgCasterJointStates_t);
    }
    if (msg_type == IMU_9DOF)
    {
//        MsgIMU9DOF_t *msg = (struct MsgIMU9DOF_t *)data_buffer;  // debug
//        LOG_F(INFO, "IMU: %d %d %d", msg->ax, msg->ay, msg->az); // debug
        bobik_zmq.send_to_zmq_topic(TOPIC_IMU9DOF, data_buffer, sizeof(MsgIMU9DOF_t));
        return sizeof(MsgIMU9DOF_t);
    }
    LOG_F(ERROR, "Unknown message type %x", msg_type);
    return 0;
}

// Read data from ZMQ
void BobikDriver::run()
{
    do
    {
        const char *topic = NULL;
        void *data = NULL;
        int data_size = 0;
        bobik_zmq.receive(topic, data, &data_size);
        if (strcmp(topic, TOPIC_CMD_VEL) == 0)
        {
            cmd_vel_callback((uint8_t *)data);
        }
        else
        {
            LOG_F(INFO, "No ZMQ topic matched for topic: %s, data: %s, size: %d\n", topic, (char *)data, data_size);
        }
    } while (!stop);
    LOG_F(INFO, "Run loop stopped");
}

void sigint_handler(int signum)
{
    LOG_F(INFO, "Received SIGnal %d. Exiting 'run' loop.\n", signum);

    stop = true;
    exit (EXIT_FAILURE); // TODO temprary
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
    std::signal(SIGINT, sigint_handler); // works but needs to unblock by receiving a message from ros2, see https://man.archlinux.org/man/zmq_setsockopt.3.en#ZMQ_CONNECT_TIMEOUT:_Set_connect()_timeout
    BobikDriver bobik_driver;
    bobik_driver.bobik_kinect.init(&(bobik_driver.bobik_zmq));
    bobik_driver.run();
    LOG_F(INFO, "Bobik_driver exiting");
    return 0;
}
