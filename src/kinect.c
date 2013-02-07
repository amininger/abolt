// General kinect lcm-broadcasting program for BOLT
// 14 Feb 2012

#include <cassert>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include <lcm/lcm.h>
#include "lcmtypes/kinect_status_t.h"

#include <libfreenect.h>
#include <cmath>

#define RGB_WIDTH 640
#define RGB_HEIGHT 480
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480

// Threading
pthread_t freenect_thread;
pthread_t accel_thread;
pthread_t lcm_thread;
int got_rgb = 0;
int got_depth = 0;

pthread_mutex_t frame_lock;// = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t accel_lock;// = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t frame_signal;// = PTHREAD_COND_INITIALIZER;

// Data buffers
uint8_t *d_buf;
uint8_t *rgb_buf;
uint8_t *d_back_buf;
uint8_t *rgb_back_buf;

// Accelerometer
double x,y,z;

// OpenKinect references
freenect_context *f_ctx;
freenect_device *f_dev;

// LCM
lcm_t *k_lcm;
int max_count = 1;
int usecs = 1000000/30;

void video_cb(freenect_device *dev, void *rgb, uint32_t ts)
{
    pthread_mutex_lock(&frame_lock);

    assert(rgb_back_buf == rgb);
    rgb_back_buf = rgb_buf;
    freenect_set_video_buffer(dev, rgb_back_buf);
    rgb_buf = (uint8_t*)rgb;

    got_rgb = 1;

    pthread_cond_signal(&frame_signal);
    pthread_mutex_unlock(&frame_lock);
}

void depth_cb(freenect_device *dev, void *depth, uint32_t ts)
{
    pthread_mutex_lock(&frame_lock);

    assert(d_back_buf == depth);
    d_back_buf = d_buf;
    freenect_set_depth_buffer(dev, d_back_buf);
    d_buf = (uint8_t*)depth;

    got_depth = 1;

    pthread_cond_signal(&frame_signal);
    pthread_mutex_unlock(&frame_lock);
}

// === Video Thread Function ================================
void *process(void *arg)
{
    // Set LED to new status to say we're running
    printf("Begin processing thread...\n");
    freenect_set_led(f_dev, LED_BLINK_GREEN);

    // Set up cameras
    printf("Setting up cameras...\n");
    freenect_set_video_callback(f_dev, video_cb);
    freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    freenect_set_video_buffer(f_dev, rgb_back_buf);
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED));
    /* freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT)); */
    freenect_set_depth_buffer(f_dev, d_back_buf);

    // Start 'er up
    printf("Kinect initialized. Starting kinect...\n");
    freenect_start_video(f_dev);
    freenect_start_depth(f_dev);

    // XXX No clean way to quit, yet
    while (freenect_process_events(f_ctx) >= 0) {
        // Process callbacks
        //usleep(usecs);
    }

    // Close everything down
    lcm_destroy(k_lcm);
    freenect_stop_video(f_dev);
    freenect_stop_depth(f_dev);
    freenect_set_led(f_dev, LED_BLINK_RED_YELLOW);

    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);

    // Free up memory
    free(d_buf);
    free(rgb_buf);

    return NULL;
}

// === Accelerometer Function =========================
void *accelup(void *arg)
{
    int hz = 30;
    printf("Begin accelerometer update thread...\n");
    while (true) {
        // Get the accelerometer state in mks units
        freenect_raw_tilt_state* state;
        freenect_update_tilt_state(f_dev);
        state = freenect_get_tilt_state(f_dev);
        freenect_get_mks_accel(state,&x,&y,&z);
        usleep(1000000/hz);

        //printf("%f %f %f\n", x, y, z);
        //fflush(stdout);
    }

}

// === LCM Thread Function ============================
void *publcm(void *arg)
{
    printf("Begin LCM publishing thread...\n");
    // Loop endlessly while waiting for data. Publish it. Repeat
    kinect_status_t ks;
    int num_sent = 0;
    int depth_bytes = (DEPTH_WIDTH*DEPTH_HEIGHT*2);
    int rgb_bytes = (RGB_WIDTH*RGB_HEIGHT*3);
    ks.rgblen = rgb_bytes;
    ks.depthlen = depth_bytes;
    //ks.rgb;
    //ks.depth;

    // XXX No clean way to quit, yet
    int herp = pthread_mutex_lock(&frame_lock);
    while (true) {
        while (!got_rgb || !got_depth) {
            pthread_cond_wait(&frame_signal, &frame_lock);
        }

        timeval time;
        gettimeofday(&time, NULL);
        ks.utime = time.tv_sec*1000 + time.tv_usec/1000;

        // Copy in arrays
        //memcpy(ks.depth, d_buf, depth_bytes);
        //memcpy(ks.rgb, rgb_buf, rgb_bytes);
        ks.depth = (int8_t*)d_buf;
        ks.rgb = (int8_t*)rgb_buf;

        // Update accelerometer data
        ks.dx = x;
        ks.dy = y;
        ks.dz = z;

        got_rgb = 0;
        got_depth = 0;
        if (++num_sent >= max_count) {
            pthread_mutex_unlock(&frame_lock);
            kinect_status_t_publish(k_lcm, "KINECT_STATUS", &ks);
            pthread_mutex_lock(&frame_lock);
            num_sent = 0;
        }
    }
    pthread_mutex_unlock(&frame_lock);
}

// ====================================================
int main(int argc, char **argv)
{
    // Specified FPS
    if (argc == 2) {
        max_count = 30/atoi(argv[1]);
        usecs = 1000000/atoi(argv[1]);
    }

    // Init pthreads
    pthread_mutex_init(&frame_lock, NULL);
    pthread_mutex_init(&accel_lock, NULL);
    pthread_cond_init(&frame_signal, NULL);

    // Init LCM
    printf("Initializing LCM...\n");
    k_lcm = lcm_create(NULL);

    if (!k_lcm)
        return 1;

    // Init buffers
    printf("Initializing buffers...\n");
    d_buf = (uint8_t*)malloc(DEPTH_WIDTH*DEPTH_HEIGHT*2);
    rgb_buf = (uint8_t*)malloc(RGB_WIDTH*RGB_HEIGHT*3);
    d_back_buf = (uint8_t*)malloc(DEPTH_WIDTH*DEPTH_HEIGHT*2);
    rgb_back_buf = (uint8_t*)malloc(RGB_WIDTH*RGB_HEIGHT*3);

    // Init kinect
    printf("Creating kinect context...\n");
    if (freenect_init(&f_ctx, NULL) < 0) {
        printf("Failed to init kinect\n");
        return 1;
    }

    // Kinect error logging & devices
    freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));


    // Detect device(s)
    int n_kinect = freenect_num_devices(f_ctx);
    printf("Found %d kinects...\n", n_kinect);
    if (n_kinect < 1) {
        printf("No kinects detected\n");
        return 1;
    }

    // Default to using 0th device.
    // Open device interface
    if (freenect_open_device(f_ctx, &f_dev, 0) < 0) {
        printf("Failed to open device\n");
        return 1;
    }

    // Create processing thread
    printf("Create kinect processing thread...\n");
    if (pthread_create(&freenect_thread, NULL, process, NULL)) {
        printf("failed to create freenect thread\n");
        return 1;
    }

    // Create accelerometer thread
    printf("Create accelerometer thread...\n");
    if (pthread_create(&accel_thread, NULL, accelup, NULL)) {
        printf("failed to create accelerometer thread\n");
        return 1;
    }

    // Create LCM thread
    printf("Create LCM processing thread...\n");
    if (pthread_create(&lcm_thread, NULL, publcm, NULL)) {
        printf("failed to create lcm thread\n");
        return 1;
    }

    pthread_exit(NULL);
    return 0;
}
