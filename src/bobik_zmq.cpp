#define ZMQ_BUILD_DRAFT_API

#include <zmq.h>
#include "zhelpers.h"
#include "loguru.hpp"
#include "protocol_types.h"
#include "bobik_zmq.hpp"

// C library headers
#include <stdio.h>
#include <string.h>

void *zmq_rep;
void *radio;
void *dish;
zmq_msg_t zmq_msg;

BobikZmq::BobikZmq()
{

    void *zmq_ctx = zmq_ctx_new();

    zmq_rep = zmq_socket(zmq_ctx, ZMQ_REP);
    zmq_connect (zmq_rep, "tcp://0.0.0.0:7555");


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

    LOG_F(INFO, "Bobik ZMQ initialized");
}

BobikZmq::~BobikZmq()
{
    LOG_F(INFO, "Bobik ZMQ finished");
}

void BobikZmq::send_to_zmq_topic(const char *topic, void *data, size_t size) const
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

/* Receive requests from 0MQ, dispach them and send synchronous response back to bobik_robot
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

// Read data from ZMQ
void BobikZmq::run()
{
    bool stop = true;
    do
    {
#ifdef ENABLE_ARDUINO
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
                void *data_buffer = zmq_msg_data(&receiveMessage);
                char *h = (char *)&receiveMessage;
                LOG_F(INFO, "CMD_VEL from ros2 topic: %s, data: %02X:%02X:%02X:%02X:%02X:%02X, size: %d\n", zmq_msg_group(&receiveMessage), h[0],h[1],h[2],h[3],h[4],h[5], bytesReceived);
                cmd_vel_callback((uint8_t *)data_buffer);
            }
            else
            {
                LOG_F(INFO, "from ros2 topic: %s, data: %s, size: %d\n", zmq_msg_group(&receiveMessage), (char *)zmq_msg_data(&receiveMessage), bytesReceived);
            }

        }

        zmq_msg_close(&receiveMessage);
#endif
    } while (!stop);
    LOG_F(INFO, "Run loop stopped");
}

