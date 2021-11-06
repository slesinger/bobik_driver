#include <future>
#include <chrono>
#include <memory>
#include <vector>

#include "bobik_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;
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
#include "uart_transporter.hpp"

#define LOOP_START1 0xEA
#define LOOP_START2 0xAA
#define LOOP_END 0xAE
#define MSG_START 0xEE

constexpr int BUFFER_SIZE = 1024;
constexpr int TTY_BAUDRATE = 500000;
constexpr int READ_POLL_MS = 100;

using namespace std::chrono_literals;

int serial_port;
struct termios tty;

BobikDriver::BobikDriver() : Node("bobik_driver")
{
    future_ = exit_signal_.get_future();
    timer_ = this->create_wall_timer(500ms, std::bind(&BobikDriver::timer_callback, this));
    pub_raw_caster_rotation = this->create_publisher<std_msgs::msg::Int16MultiArray>("driver/raw/caster", 10);
    sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&BobikDriver::cmd_vel_callback, this, _1));

    device = "/dev/ttyUSB0"; // TODO discover
    transporter_ = std::make_unique<UARTTransporter>(device, TTY_BAUDRATE, READ_POLL_MS, BUFFER_SIZE);
    if (transporter_->init() < 0)
    {
        throw std::runtime_error("Failed to initialize transport");
    }

    read_thread_ = std::thread(&BobikDriver::read_thread_func, this, future_);
    RCLCPP_INFO(this->get_logger(), "Bobik driver initialized");
}

BobikDriver::~BobikDriver()
{
    transporter_->close();
    RCLCPP_INFO(this->get_logger(), "Bobik driver finished"); // TODO this is not called. See log '[rclcpp]: signal_handler(signal_value=2)'
}

void BobikDriver::timer_callback()
{
    uint8_t vels[6] = {11, 12, 13, 14, 15, 16};
    int written = 0;
    written = transporter_->write(DRIVE_CMD, vels, 6);
    // printf("time");  //does not work here
    RCLCPP_INFO(this->get_logger(), "Written to arduino %d", written); //loggin does not work here
}

void BobikDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
    geometry_msgs::msg::Vector3 linear = msg->linear;
    // geometry_msgs::msg::Vector3 angular = msg->angular;
    //angular
    printf("vel");
    rclcpp::shutdown();
    RCLCPP_INFO(this->get_logger(), "X: '%f'", linear.x);
}

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
            RCLCPP_INFO(this->get_logger(), "Received bytu ----%d ", length);
            char *bufo = (char *)(data_buffer.get());
            char bufs[1000];
            char *bufc = bufs;
            for (int i = 0; i < length; i++)
            {
                sprintf(bufc, "%02x:", (uint8_t)bufo[i]);
                bufc += 3;
            }
            *bufc = 0;
            // RCLCPP_INFO(this->get_logger(), "Loop ------- %d| %s", length, bufs);
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
            RCLCPP_ERROR(this->get_logger(), "Expected MSG_START but got something else %x", *p);
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
        // MsgTimestamp_t *msg = (struct MsgTimestamp_t *)data_buffer;
        // RCLCPP_INFO(this->get_logger(), "Time %d", msg->timestamp);
        return sizeof(MsgTimestamp_t);
    }
    if (msg_type == LOOP_END_MSG)
    {
        // MsgCRC_t *msg = (struct MsgCRC_t *)data_buffer;
        // RCLCPP_INFO(this->get_logger(), "CRC %x", msg->crc);
        return sizeof(MsgCRC_t);
    }
    if (msg_type == LOG4)
    {
        MsgLog4_t *msg = (struct MsgLog4_t *)data_buffer;
        RCLCPP_INFO(this->get_logger(), "LOG %x %x %x %x", msg->log1, msg->log2, msg->log3, msg->log4);
        return sizeof(MsgLog4_t);
    }
    if (msg_type == LOADCELL_UPPER_ARM_LIFT_JOINT)
    {
        MsgLoadCell_t *msg = (struct MsgLoadCell_t *)data_buffer;
        RCLCPP_INFO(this->get_logger(), "Load %d", msg->value);
        return sizeof(MsgLoadCell_t);
    }
    if (msg_type == CASTER_JOINT_STATES)
    {
        MsgCasterJointStates_t *msg = (struct MsgCasterJointStates_t *)data_buffer;
        std::vector<int16_t> data = {msg->fl_caster_rotation_joint, msg->fr_caster_rotation_joint, msg->r_caster_rotation_joint};
        auto message = std_msgs::msg::Int16MultiArray();
        message.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
        message.layout.dim[0].label = "rot";
        message.layout.dim[0].size = data.size();
        message.layout.dim[0].stride = 1;
        message.layout.data_offset = 0;
        message.data = data;
        pub_raw_caster_rotation->publish(message);
        // RCLCPP_INFO(this->get_logger(), "Caster rotation %d : %d : %d", msg->fl_caster_rotation_joint, msg->fr_caster_rotation_joint, msg->r_caster_rotation_joint);
        return sizeof(MsgCasterJointStates_t);
    }
    RCLCPP_ERROR(this->get_logger(), "Unknown message type %x", msg_type);
    return 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BobikDriver>());
    rclcpp::shutdown();
    return 0;
}
