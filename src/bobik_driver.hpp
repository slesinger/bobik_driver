#ifndef BOBIK_DRIVER_HPP_
#define BOBIK_DRIVER_HPP_

#include "uart_transporter.hpp"
#include "bobik_kinect.hpp"
#include "bobik_zmq.hpp"

class BobikDriver
{
public:
    BobikDriver();
    ~BobikDriver();
    void run();
    BobikZmq bobik_zmq;
    BobikKinect bobik_kinect;

private:
    unsigned char state = 0;
    int payload_countdown = 0; // how many bytes to read to complete payload
    unsigned char payload_buf[32];
    unsigned long loop_ts = 0;
    std::unique_ptr<Transporter> transporter_;
    std::string backend_comms{};
    std::string device{};
    uint32_t baudrate;
    int64_t dynamic_serial_mapping_ms{-1};
    uint32_t read_poll_ms;
    size_t ring_buffer_size;
    std::shared_future<void> future_;
    std::promise<void> exit_signal_;
    std::thread read_from_arduino_thread_;
    std::thread read_from_lidar_thread_;
    std::thread read_from_kinect_thread_;
    std::thread serve_reqresp_thread_;

    void cmd_vel_callback(uint8_t *msg_cmd_vel) const;
    void read_from_lidar_thread_func(const std::shared_future<void> &local_future);
    void read_from_kinect_thread_func(const std::shared_future<void> &local_future);
    void serve_reqresp_thread_func(const std::shared_future<void> &local_future);
    void read_from_arduino_thread_func(const std::shared_future<void> &local_future);
    void dispatch_from_arduino(uint8_t *data_buffer, ssize_t length);
    int dispatch_msg_from_arduino(uint8_t msg_type, uint8_t *data_buffer);

    size_t count_;

};

#endif