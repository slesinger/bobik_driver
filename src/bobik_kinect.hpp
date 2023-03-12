#ifndef BOBIK_KINECT
#define BOBIK_KINECT

#include "libfreenect.h"
#include "bobik_zmq.hpp"
#include <stdio.h>
#include <zstd.h>
#include <cstdlib>

typedef struct {
    void* fBuffer;
    void* cBuffer;
    size_t fBufferSize;
    size_t cBufferSize;
    ZSTD_CCtx* cctx;
} resources;

/*
 * Define the returned error code from utility functions.
 */
typedef enum {
    ERROR_fsize = 1,
    ERROR_fopen = 2,
    ERROR_fclose = 3,
    ERROR_fread = 4,
    ERROR_fwrite = 5,
    ERROR_loadFile = 6,
    ERROR_saveFile = 7,
    ERROR_malloc = 8,
    ERROR_largeFile = 9,
} COMMON_ErrorCode;

/*! CHECK
 * Check that the condition holds. If it doesn't print a message and die.
 */
#define CHECK(cond, ...)                        \
    do {                                        \
        if (!(cond)) {                          \
            fprintf(stderr,                     \
                    "%s:%d CHECK(%s) failed: ", \
                    __FILE__,                   \
                    __LINE__,                   \
                    #cond);                     \
            fprintf(stderr, "" __VA_ARGS__);    \
            fprintf(stderr, "\n");              \
            exit(1);                            \
        }                                       \
    } while (0)

/*! CHECK_ZSTD
 * Check the zstd error code and die if an error occurred after printing a
 * message.
 */
#define CHECK_ZSTD(fn, ...)                                      \
    do {                                                         \
        size_t const err = (fn);                                 \
        CHECK(!ZSTD_isError(err), "%s", ZSTD_getErrorName(err)); \
    } while (0)



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

    int send_rgb();

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
    static void rgb_cb(freenect_device* dev, void* data, uint32_t timestamp);
    static void rgb_cbX(freenect_device* dev, void* data, uint32_t timestamp);
    void createResources_orDie();

};
#endif
