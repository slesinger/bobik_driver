#include "bobik_kinect.hpp"
#include <future>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "libfreenect.h"
#include "loguru.hpp"
#include "protocol_types.h"

freenect_device* fn_dev;
freenect_context* fn_ctx;

BobikZmq *bzmq;

void BobikKinect::depth_cb(freenect_device* dev, void* data, uint32_t timestamp)
{
//    printf("Received depth frame at %d\n", timestamp);
//    uint16_t *depth = (uint16_t*)data;

    for (int line = 0; line < 480; line++)
    {
  //      bzmq->send_to_zmq_topic(TOPIC_KINECT_DEPTH, data + line*640*sizeof(uint16_t), 640 * sizeof(uint16_t));
    }
}

void video_cb(freenect_device* dev, void* data, uint32_t timestamp)
{
//    printf("Received video frame at %d\n", timestamp);
}


BobikKinect::BobikKinect()
{
    initialized = false;
}

BobikKinect::~BobikKinect()
{
}

int BobikKinect::init(BobikZmq *zmq)
{
    bzmq = zmq;

        // Initialize libfreenect.
        int ret = freenect_init(&fn_ctx, NULL);
        if (ret < 0)
            return ret;

        // Show debug messages and use camera only.
        freenect_set_log_level(fn_ctx, FREENECT_LOG_DEBUG);
        freenect_select_subdevices(fn_ctx, FREENECT_DEVICE_CAMERA);

        // Find out how many devices are connected.
        int num_devices = ret = freenect_num_devices(fn_ctx);
        if (ret < 0)
            return ret;
        if (num_devices == 0)
        {
            printf("No device found!\n");
            freenect_shutdown(fn_ctx);
            return 1;
        }

        // Open the first device.
        ret = freenect_open_device(fn_ctx, &fn_dev, 0);
        if (ret < 0)
        {
            freenect_shutdown(fn_ctx);
            return ret;
        }

        // Set depth and video modes.
        ret = freenect_set_depth_mode(fn_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM));
        if (ret < 0)
        {
            freenect_shutdown(fn_ctx);
            return ret;
        }
        ret = freenect_set_video_mode(fn_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
        if (ret < 0)
        {
            freenect_shutdown(fn_ctx);
            return ret;
        }

        // Set frame callbacks.
        freenect_set_depth_callback(fn_dev, BobikKinect::depth_cb);
        freenect_set_video_callback(fn_dev, video_cb);

        LOG_F(INFO, "Starting Kinect streams");
        // Start depth and video.
        ret = freenect_start_depth(fn_dev);
        if (ret < 0)
        {
            freenect_shutdown(fn_ctx);
            return ret;
        }
        ret = freenect_start_video(fn_dev);
        if (ret < 0)
        {
            freenect_shutdown(fn_ctx);
            return ret;
        }
        LOG_F(INFO, "Kinect started. Number of devices %d", num_devices);
        initialized = true;
        return 0;
    }

void BobikKinect::shutdown()
{
        // Stop everything and shutdown.
        freenect_stop_depth(fn_dev);
        freenect_stop_video(fn_dev);
        freenect_close_device(fn_dev);
        freenect_shutdown(fn_ctx);
        LOG_F(INFO, "Kinect stopped");
}

int BobikKinect::run()
{
    if (!initialized) return 0;

    return freenect_process_events(fn_ctx);  // process USB events
}


