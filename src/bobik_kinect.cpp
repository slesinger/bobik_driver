#include "bobik_kinect.hpp"
#include <future>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <zstd.h>
#include "libfreenect.h"
#include "loguru.hpp"
#include "protocol_types.h"
#include <sys/time.h>

freenect_device* fn_dev;
freenect_context* fn_ctx;

#define WIDTH 640
#define HEIGHT 480
#define RGB_MULTILINE 1  //must be HEIGHT divisible by this. 30 is max with best compr ration 1.37. Level 1 has compr 1.33

uint8_t frame_rgb[WIDTH * HEIGHT * 3];
bool new_rgb = false;

BobikZmq *bzmq;

resources res_depth;
resources res_rgb;


int cnt_usb_dpt = 0;
int cnt_usb_rgb = 0;
int cnt_snd_dpt = 0;
int cnt_snd_rgb = 0;
unsigned long sum_org_rgb = 0;
unsigned long sum_cmp_rgb = 0;

    /*! malloc_orDie() :
     * Allocate memory.
     *
     * @return If successful this function returns a pointer to allo-
     * cated memory.  If there is an error, this function will send that
     * error to stderr and exit.
     */
static void* malloc_orDie(size_t size)
{
    void* const buff = malloc(size);
    if (buff) return buff;
    /* error */
    perror("malloc");
    exit(ERROR_malloc);
}


// Transmit 2 lines at a time
void BobikKinect::depth_cb(freenect_device* dev, void* data, uint32_t timestamp)
{
    uint16_t *depth_line_buff = (uint16_t *)res_depth.fBuffer;
    for (uint16_t line = 0; line < HEIGHT; line+=2)
    {
        depth_line_buff[0] = line; //first message word is line id
        memcpy((void *)(&depth_line_buff[1]), data + line*WIDTH*sizeof(uint16_t), 2 * WIDTH * sizeof(uint16_t));

/*        size_t const cSize = ZSTD_compressCCtx(
                    res_depth.cctx,
                    res_depth.cBuffer,
                    res_depth.cBufferSize,
                    res_depth.fBuffer,
                    res_depth.fBufferSize,
                    1
        );
        CHECK_ZSTD(cSize);
        bzmq->send_to_zmq_topic_kinect(TOPIC_KINECT_DEPTH, res_depth.cBuffer, cSize);*/
    }

    cnt_usb_dpt++;
    if (cnt_usb_dpt >= 30) {
        printf("Received depth %d, rgb %d. Sent depth %d, rgb %d.\n", cnt_usb_dpt, cnt_usb_rgb, cnt_snd_dpt, cnt_snd_rgb);
        cnt_usb_dpt = 0;
        cnt_usb_rgb = 0;
        cnt_snd_dpt = 0;
        cnt_snd_rgb = 0;
    }

}


void BobikKinect::rgb_cbX(freenect_device* dev, void* data, uint32_t timestamp)
{

    uint8_t *rgb_line_buff = (uint8_t *)res_rgb.fBuffer;
    for (uint16_t line = 0; line < HEIGHT; line++)
    {
        //first message word is line id
        rgb_line_buff[0] = line & 0XFF;
        rgb_line_buff[1] = line >> 8 & 0XFF;
        memcpy((void *)(&rgb_line_buff[2]), data + line*WIDTH*3*sizeof(uint8_t), WIDTH * 3 * sizeof(uint8_t));
    }
}

void BobikKinect::rgb_cb(freenect_device* dev, void* data, uint32_t timestamp)
{
    if (new_rgb == false) // previous frame was sent out, let's accept new one from kinect
    {
        memcpy((void *)frame_rgb, data, WIDTH * HEIGHT * 3 * sizeof(uint8_t));
        cnt_usb_rgb++;
        new_rgb = true;

    }
}

struct outMsg {
    uint16_t line;
    uint16_t multiline;
    uint8_t line_buff[RGB_MULTILINE * WIDTH * 3 +70];   //compressed muze byt az o 70 bytu vetsi nez uncompressed, muse se pouzit ZSTD_compressBound() na zjisteni
};

int BobikKinect::send_rgb()
{
    if (new_rgb == true)
    {
struct timeval stop, start;
        outMsg rgb;
        rgb.multiline = RGB_MULTILINE;
        for (uint16_t line = 0; line < HEIGHT; line += RGB_MULTILINE)
        {
            //first message word is line id
//            memset(res_rgb.fBuffer, line%256, WIDTH*3);
            rgb.line = line;
//            *(res_rgb.fBuffer) = line & 0XFF;
//            (uint8_t *)(&res_rgb.fBuffer[1]) = line >> 8 & 0XFF;
//            rgb_line_buff[0] = line & 0XFF;
//            rgb_line_buff[1] = line >> 8 & 0XFF;
//            memcpy((void *)(&rgb_line_buff[2]), frame_rgb + line*WIDTH*3*sizeof(uint8_t), WIDTH * 3 * sizeof(uint8_t));
gettimeofday(&start, NULL);
            size_t const cSize = ZSTD_compressCCtx(
                    res_rgb.cctx,
                    res_rgb.cBuffer,
                    res_rgb.cBufferSize,
                    frame_rgb + line*WIDTH*3*sizeof(uint8_t),
                    res_rgb.fBufferSize,
                    15
            );
gettimeofday(&stop, NULL);
            CHECK_ZSTD(cSize);
            memcpy(rgb.line_buff, res_rgb.cBuffer, cSize);
            bzmq->send_to_zmq_topic_kinect_rgb(TOPIC_KINECT_RGB, (void *)(&rgb), cSize + sizeof(rgb.line) + sizeof(rgb.multiline));
//            printf("%d size %d > %d compress time %lu us\n", rgb.line, res_rgb.fBufferSize, cSize, (stop.tv_sec - start.tv_sec) * 1000000 + stop.tv_usec - start.tv_usec);
            for (int i=0; i<100000; i++) {}  // TODO nhradit cekanim nebo lepsi kompresi
            bzmq->send_to_zmq_topic_kinect_rgb(TOPIC_KINECT_RGB, res_rgb.fBuffer, res_rgb.fBufferSize);  //no compression
            sum_org_rgb += res_rgb.fBufferSize;
            sum_cmp_rgb += cSize;
//printf("took %lu us\n", (stop.tv_sec - start.tv_sec) * 1000000 + stop.tv_usec - start.tv_usec);
        }
        cnt_snd_rgb++;

        printf("Frame. orig %d comp %d ratio %f, %lu us\n", sum_org_rgb/HEIGHT, sum_cmp_rgb/HEIGHT, (float)(sum_org_rgb/HEIGHT) / (float)(sum_cmp_rgb/HEIGHT), (stop.tv_sec - start.tv_sec) * 1000000 + stop.tv_usec - start.tv_usec );
        sum_org_rgb = 0;
        sum_cmp_rgb = 0;

        new_rgb = false;
        return 1;
    }
    return 0;
}

BobikKinect::BobikKinect()
{
    initialized = false;
    createResources_orDie();
}

BobikKinect::~BobikKinect()
{
    free(res_depth.fBuffer);
    free(res_depth.cBuffer);
    ZSTD_freeCCtx(res_depth.cctx); 

    free(res_rgb.fBuffer);
    free(res_rgb.cBuffer);
    ZSTD_freeCCtx(res_rgb.cctx); 
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
        freenect_set_video_callback(fn_dev, BobikKinect::rgb_cb);

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

/*
 * allocate memory for buffers big enough to compress all files
 * as well as memory for output file name (ofn)
 */
void BobikKinect::createResources_orDie()
{
    res_depth.fBufferSize = sizeof(uint16_t) * (2 * WIDTH + 1);
    res_depth.cBufferSize = ZSTD_compressBound(res_depth.fBufferSize);
    res_depth.fBuffer = malloc_orDie(res_depth.fBufferSize);
    res_depth.cBuffer = malloc_orDie(res_depth.cBufferSize);
    res_depth.cctx = ZSTD_createCCtx();
    CHECK(res_depth.cctx != NULL, "ZSTD_createCCtx() depth failed!");
    res_rgb.fBufferSize = sizeof(uint8_t) * RGB_MULTILINE * WIDTH * 3;
    res_rgb.cBufferSize = ZSTD_compressBound(res_rgb.fBufferSize);
    res_rgb.fBuffer = malloc_orDie(res_rgb.fBufferSize);
    res_rgb.cBuffer = malloc_orDie(res_rgb.cBufferSize);
    res_rgb.cctx = ZSTD_createCCtx();
    CHECK(res_rgb.cctx != NULL, "ZSTD_createCCtx() depth failed!");
printf("fbufferSize: %d, cBufferSize: %d\n", res_rgb.fBufferSize, res_rgb.cBufferSize);
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


