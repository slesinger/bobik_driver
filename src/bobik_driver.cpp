#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include <iostream>
#include "protocol_types.h"

#define LOOP_START1 0xEA
#define LOOP_START2 0xAA
#define LOOP_END 0xAE
#define MSG_START 0xEE

using namespace std::chrono_literals;
// using namespace std;

int serial_port;
struct termios tty;

// #define STATE_PAYLOAD_READ 0
// #define MSGTYPE_UNDEFINED 0

class BobikDriver : public rclcpp::Node
{
public:
  BobikDriver()
      : Node("bobik_driver"), count_(0)
  {
    timer_ = this->create_wall_timer(500ms, std::bind(&BobikDriver::timer_callback, this));
    publisher_ = this->create_publisher<std_msgs::msg::String>("bobik1", 10);
    sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&BobikDriver::cmd_vel_callback, this, _1));
  }

private:
  unsigned char state = 0;
  int payload_countdown = 0; //how many bytes to read to complete payload
  unsigned char payload_buf[32];
  unsigned long loop_ts = 0;

  unsigned long read_ulong(unsigned char *payload)
  {
    unsigned long v = *payload;
    v <<= 8;
    v += *(payload + 1);
    v <<= 8;
    v += *(payload + 2);
    v <<= 8;
    v += *(payload + 3);
    return v;
  }

  long read_long(unsigned char *payload)
  {
    long v = *payload;
    v <<= 8;
    v += *(payload + 1);
    v <<= 8;
    v += *(payload + 2);
    v <<= 8;
    v += *(payload + 3);
    return v;
  }

  void process_message(unsigned char msg_type, unsigned char *payload)
  {
    switch (msg_type)
    { // Message type
    case LOADCELL_UPPER_ARM_LIFT_JOINT:
    {
      long load = read_long(payload);
      payload += 4;
      RCLCPP_INFO(this->get_logger(), "loadcell is: '%d'", load);
      break;
    }
    case LOOP_TIMESTAMP:
    {
      loop_ts = read_ulong(payload);
      payload += 4;
      RCLCPP_INFO(this->get_logger(), "timestamp is: '%d'", loop_ts);
      break;
    }
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown process message type: '%x'", msg_type);
    }
  }

  int lookup_payload_length(unsigned char msg_type)
  {
    switch (msg_type) // Message type
    {
    case LOADCELL_UPPER_ARM_LIFT_JOINT:
      return 4;
      break;
    case LOOP_TIMESTAMP:
      return 4;
      break;
    }
    RCLCPP_ERROR(this->get_logger(), "Unknown message type: '%x'", msg_type);
    return 0;
  }

  void timer_callback()
  {
    unsigned char read_buf[256];
    int n = read(serial_port, &read_buf, sizeof(read_buf));
    RCLCPP_INFO(this->get_logger(), "New spin: '%d'", n);
    if (n == 0)
      return;

    // Expected to read payload?
    for (unsigned char *p = read_buf; p <= read_buf + n; ++p)
    {
      // RCLCPP_INFO(this->get_logger(), "Data: '%x', state: %x", *p, state);

      if (payload_countdown > 0)
      {
        payload_countdown--;
        payload_buf[payload_countdown] = *p;
        if (payload_countdown == 0)
        {
          process_message(state, payload_buf);
          state = 0;
        }
        continue;
      }

      // Expected message type?
      if (state == MSG_START)
      {
        payload_countdown = lookup_payload_length(*p);
        state = *p; //message_type
        continue;
      }

      if (state == LOOP_END)
      {
        RCLCPP_INFO(this->get_logger(), "Loop end with CRC8: '%x'", *p);
        payload_countdown = 0;
        state = 0;
        continue;
      }

      switch (*p)
      {
      case LOOP_START1:
        state = LOOP_START;
        break;

      case LOOP_END:
        state = LOOP_END;
        break;

      case MSG_START:
        state = MSG_START;
        break;

      default: // read value
        RCLCPP_WARN(this->get_logger(), "Unknown char or position from serial: '%x'", *p);
      }
    }

    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++) + " - " + std::to_string(n);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    // RCLCPP_INFO(this->get_logger(), "Received from TTY: '%x %x %x %x %x %x %x %x'", read_buf[0], read_buf[1], read_buf[2], read_buf[3], read_buf[4], read_buf[5], read_buf[6], read_buf[7]);
    // publisher_->publish(message);
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    geometry_msgs::msg::Vector3 linear = msg->linear;
    // geometry_msgs::msg::Vector3 angular = msg->angular;
    //angular
    RCLCPP_INFO(this->get_logger(), "X: '%f'", linear.x);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

void setup()
{
  serial_port = open("/dev/ttyUSB0", O_RDWR);

  if (serial_port < 0)
  {
    printf("Error %i from open: %s. Is Arduino connected?\n", errno, strerror(errno));
    exit(1);
  }

  if (tcgetattr(serial_port, &tty) != 0)
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    exit(2);
  }

  //https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
  tty.c_cflag &= ~PARENB;        // Clear parity bit
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication
  tty.c_cflag |= CS8;            // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines
  // tty.c_lflag |= ICANON;         //enable cannonical mode to receive loop message upon \n
  tty.c_lflag &= ~ICANON; //enable cannonical mode to receive loop message upon \n
  tty.c_lflag &= ~ECHO;   // Disable echo
  tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IGNBRK | BRKINT |
                   PARMRK | ISTRIP |
                   INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_oflag &= ~OPOST;                   // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR;                   // Prevent conversion of newline to carriage return/line feed
  tty.c_cc[VTIME] = 0;                     // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;
  cfsetispeed(&tty, B115200); //B460800
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
  {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }
}

void unsetup()
{
  close(serial_port);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  setup();
  rclcpp::spin(std::make_shared<BobikDriver>());
  unsetup();
  rclcpp::shutdown();
  return 0;
}
