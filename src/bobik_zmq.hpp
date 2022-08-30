#ifndef BOBIK_ZMQ_HPP_
#define BOBIK_ZMQ_HPP_

#include <stddef.h>

class BobikZmq
{
public:
    BobikZmq();
    ~BobikZmq();
    void run();
    void send_to_zmq_topic(const char *topic, void *data, size_t size) const;
    void send_to_zmq_topic_kinect(const char *topic, void *data, size_t size) const;
    void receive(const char *topic, void *data, int *data_size) const;

private:
    uint16_t udp_send_port{0};
    uint16_t udp_recv_port{0};
//    std::thread serve_reqresp_thread_;
};

#endif