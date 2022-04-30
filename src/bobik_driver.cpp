#define ZMQ_BUILD_DRAFT_API
#include <future>
#include <chrono>
#include <memory>
#include <vector>
#include <zmq.hpp>

#include "bobik_driver.hpp"
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

using namespace std::chrono_literals;

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

    read_thread_ = std::thread(&BobikDriver::read_thread_func, this, future_);
    LOG_F(INFO, "Bobik driver initialized");
}

BobikDriver::~BobikDriver()
{
    LOG_F(INFO, "Bobik driver finished"); // TODO this is not called. See log '[rclcpp]: signal_handler(signal_value=2)'
    transporter_->close();
}

void BobikDriver::send_to_zmq_topic(const char *topic, uint8_t *data, size_t size) const
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
 * Reads data from the serial port and publishes it as a ROS message.
 */
void BobikDriver::read_thread_func(const std::shared_future<void> &local_future)
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
            // LOG_F(INFO, "Loop ------- %d| %s", length, bufs);
            dispatch(data_buffer.get(), length);
        }
        status = local_future.wait_for(std::chrono::seconds(0));
    } while (status == std::future_status::timeout);
}

void BobikDriver::dispatch(uint8_t *data_buffer, ssize_t length)
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
        int msg_len = callback(msg_type, p);
        p += msg_len;
    } while (p < (data_buffer + length));
}

int BobikDriver::callback(uint8_t msg_type, uint8_t *data_buffer)
{
    if (msg_type == LOOP_TIMESTAMP)
    {
        MsgTimestamp_t *msg = (struct MsgTimestamp_t *)data_buffer;
        LOG_F(INFO, "Time %d", msg->timestamp);
        return sizeof(MsgTimestamp_t);
    }
    if (msg_type == LOOP_END_MSG)
    {
        MsgCRC_t *msg = (struct MsgCRC_t *)data_buffer;
        LOG_F(INFO, "CRC %x", msg->crc);
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
        send_to_zmq_topic("caster_raw_joint_states", data_buffer, sizeof(MsgCasterJointStates_t));
        return sizeof(MsgCasterJointStates_t);
    }
    LOG_F(ERROR, "Unknown message type %x", msg_type);
    return 0;
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
    BobikDriver bobik_driver;
    return 0;
}
