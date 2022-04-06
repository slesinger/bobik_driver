#include <future>
#include <chrono>
#include <memory>
#include <vector>

#include "bobik_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
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
    pub_raw_caster_drive = this->create_publisher<std_msgs::msg::Int16MultiArray>("driver/raw/wheel", 10);
    pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
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
    // uint8_t vels[6] = {11, 12, 13, 14, 15, 16};
    // int written = 0;
    // written = transporter_->write(DRIVE_CMD, vels, 6);
    // RCLCPP_INFO(this->get_logger(), "Written to arduino %d", written);
}

void BobikDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
    geometry_msgs::msg::Vector3 linear = msg->linear;
    geometry_msgs::msg::Vector3 angular = msg->angular;
    // RCLCPP_INFO(this->get_logger(), "X: '%f'", linear.x);
    MsgCmdVel_t msg_cmd_vel;
    // int16_t rot = 0;
    // int16_t drive = (int16_t)(linear.x * 80);
    msg_cmd_vel.linear_x = (int16_t)(linear.x * FLOAT_INT16_PRECISION);
    msg_cmd_vel.linear_y = (int16_t)(linear.y * FLOAT_INT16_PRECISION);
    msg_cmd_vel.rotation = (int16_t)(angular.z * FLOAT_INT16_PRECISION);
    int written = transporter_->write(DRIVE_CMD, (uint8_t*)(&msg_cmd_vel), 6);
    if (written <= 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to arduino, %d", written);
    }
    // RCLCPP_INFO(this->get_logger(), "msg_cmd_vel written bytes: '%d'", written);

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
            // RCLCPP_INFO(this->get_logger(), "Received bytes ----%d ", length);
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
        pub_odom->publish(calculate_odom(&data));
        // RCLCPP_INFO(this->get_logger(), "Caster rotation %d : %d : %d", msg->fl_caster_rotation_joint, msg->fr_caster_rotation_joint, msg->r_caster_rotation_joint);
        return sizeof(MsgCasterJointStates_t);
    }
    RCLCPP_ERROR(this->get_logger(), "Unknown message type %x", msg_type);
    return 0;
}

nav_msgs::msg::Odometry BobikDriver::calculate_odom(std::vector<int16_t> *data)
{
    #define LEN_AB 0.457 // distance between twa caster axis
    #define LEN_AB_HALF LEN_AB / 2.0
    #define LEN_Cc LEN_AB * 1.732051 / 2.0 // height of same-side triangle, distance between point C and center of line c (AB)
    #define LEN_SC 2.0 / 3.0 * LEN_Cc  //distance between robot center (S - stred) and rear caster axis C
    #define LEN_Sc 1.0 / 3.0 * LEN_Cc  //distance between robot center (S - stred) and center of line c (AB)
    #define POS_A_x LEN_Sc
    #define POS_A_y -LEN_AB_HALF
    #define POS_B_x LEN_Sc
    #define POS_B_y LEN_AB_HALF
    #define POS_C_x -LEN_SC
    #define POS_C_y 0.0
    #define CASTER_UNITS2RAD M_PI / -8192.0
    #define CASTER_TICKS2METERS (0.123 * M_PI) / (2*120) 

    MsgCasterJointStates_t *caster_data = (struct MsgCasterJointStates_t *)data->data();
    // Front Left caster new position
    float Arad = CASTER_UNITS2RAD * caster_data->fl_caster_rotation_joint;
    float Ameters = CASTER_TICKS2METERS * caster_data->fl_caster_drive_joint;
    float Ax = POS_A_x + cos(Arad) * Ameters;
    float Ay = POS_A_y - sin(Arad) * Ameters;
    // Front Right caster new position
    float Brad = CASTER_UNITS2RAD * caster_data->fr_caster_rotation_joint;
    float Bmeters = CASTER_TICKS2METERS * caster_data->fr_caster_drive_joint;
    float Bx = POS_B_x + cos(Brad) * Bmeters;
    float By = POS_B_y - sin(Brad) * Bmeters;
    // Rear caster new position
    float Crad = CASTER_UNITS2RAD * caster_data->r_caster_rotation_joint;
    float Cmeters = CASTER_TICKS2METERS * caster_data->r_caster_drive_joint;
    float Cx = POS_C_x + cos(Crad) * Cmeters;
    float Cy = POS_C_y - sin(Crad) * Cmeters;
    // Center position between front left and right
    float c_center_x = (Ax + Bx) / 2.0;
    float c_center_y = (Ay + By) / 2.0;
    // Vector between c center and rear
    float c_center_to_rear_x = Cx - c_center_x;
    float c_center_to_rear_y = Cy - c_center_y;
    // New base center position
    float base_center_x = c_center_x + c_center_to_rear_x / 3.0;
    float base_center_y = c_center_y + c_center_to_rear_y / 3.0;
    // New base vector forward
    float base_forward_x = -c_center_to_rear_x;
    float base_forward_y = -c_center_to_rear_y;
    // Rotation of base in radians
    float base_rotation = atan2(base_forward_y, base_forward_x); //assumption: base cannot rotate more than 90 degrees with one frame

    odom_pose.position.x += base_center_x;
    odom_pose.position.y += base_center_y;
    odom_pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, base_rotation);
    odom_pose.orientation = tf2::toMsg(q);

    nav_msgs::msg::Odometry message;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.header.stamp = rclcpp::Clock().now();
    message.pose.pose = odom_pose;
    message.twist.twist.linear.x = base_center_x;
    message.twist.twist.linear.y = base_center_y;
    message.twist.twist.linear.z = 0;
    message.twist.twist.angular.x = 0;
    message.twist.twist.angular.y = 0;
    message.twist.twist.angular.z = base_rotation;
    return message;
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
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BobikDriver>());
    rclcpp::shutdown();
    return 0;
}
