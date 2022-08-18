#ifndef BOBIK_KINECT
#define BOBIK_KINECT

#include "libfreenect.h"
#include "bobik_zmq.hpp"

class BobikKinect
{
public:
    /**
     * @brief Initialize 1st Kinect device for depth+rgb
     */
    BobikKinect();
    /**
     * @brief Default destructor
     */
    ~BobikKinect();

    /**
     * Setup Kinect device
     */
    int init(BobikZmq *zmq);

    /**
     * Called from thread function that does the main loop for processing Kinect USB events
     */
    int run();

    /**
     * Main process registers signals and sets this property to true in case SIGINT is catched.
     */
    void shutdown();
//    static BobikZmq* bzmq;

private:
    bool initialized;
    freenect_context* fn_ctx;
    freenect_device* fn_dev;
    static void depth_cb(freenect_device* dev, void* data, uint32_t timestamp);
};
#endif
