#ifndef BOBIK_DRIVER_HPP_
#define BOBIK_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "uart_transporter.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class BobikDriver : public rclcpp::Node
{
public:
    BobikDriver();
    ~BobikDriver();

private:
    unsigned char state = 0;
    int payload_countdown = 0; //how many bytes to read to complete payload
    unsigned char payload_buf[32];
    unsigned long loop_ts = 0;
    std::unique_ptr<Transporter> transporter_;
    std::string backend_comms{};
    std::string device{};
    uint32_t baudrate;
    int64_t dynamic_serial_mapping_ms{-1};
    uint32_t read_poll_ms;
    size_t ring_buffer_size;
    uint16_t udp_send_port{0};
    uint16_t udp_recv_port{0};
    std::shared_future<void> future_;
    std::promise<void> exit_signal_;
    std::thread read_thread_;

    void timer_callback();
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;
    void read_thread_func(const std::shared_future<void> &local_future);
    void dispatch(uint8_t *data_buffer, ssize_t length);
    int callback(uint8_t msg_type, uint8_t *data_buffer);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_raw_caster_rotation;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_raw_caster_drive;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    geometry_msgs::msg::Pose odom_pose;
    /**
     * @brief Calculate odometry from drive joint states.
     * 
     * @param MsgCasterJointStates_t data 
     * @return nav_msgs::msg::Odometry message
     */
    nav_msgs::msg::Odometry calculate_odom(std::vector<int16_t> *data);

    size_t count_;
};

#endif