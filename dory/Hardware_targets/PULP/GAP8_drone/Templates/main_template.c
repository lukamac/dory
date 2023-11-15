/*
 * test_template.c
 * Alessio Burrello <alessio.burrello@unibo.it>
 *
 * Copyright (C) 2019-2020 University of Bologna
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */
<%
l3_supported = DORY_HW_graph[0].HW_description['memory']['levels'] > 2
n_inputs = DORY_HW_graph[0].n_test_inputs
%>\
% if not l3_supported:
#include "${prefix}input.h"
% else:
#include "mem.h"
% endif
#include "${prefix}network.h"
#include "pmsis.h"

#include "bsp/bsp.h"
#include "bsp/buffer.h"
#include "bsp/camera/himax.h"
#include "bsp/display/ili9341.h"
#include "bsp/flash/hyperflash.h"
#include "bsp/ram.h"
#include "bsp/ram/hyperram.h"
#include "pmsis.h"

#include "Gap.h"
#include "gaplib/ImgIO.h"

#include "bsp/flash/hyperflash.h"
#include "bsp/fs.h"
#include "bsp/fs/readfs.h"
#include "bsp/transport/nina_w10.h"
#include "tools/frame_streamer.h"

% if verbose:
#define VERBOSE 1
% endif
#define PERF 1 // print FPS performances of the network

// Change Mode
// #define REGRESSION_AS_CLASSIFICATION 1
// #define IMAV 1

// Defines
#define FREQ_FC 200
#define FREQ_CL 175

// Camera
#ifdef IMAV
#define CAMERA_WIDTH 162
#define CAMERA_HEIGHT 162
#define INPUT_WIDTH 162
#define INPUT_HEIGHT 162
#define INPUT_COLORS 1
#else
#define CAMERA_WIDTH 324
#define CAMERA_HEIGHT 244
#define INPUT_WIDTH 200
#define INPUT_HEIGHT 200
#define INPUT_COLORS 1
#endif

#define CAMERA_SIZE (CAMERA_HEIGHT * CAMERA_WIDTH)
#define BUFF_SIZE (CAMERA_WIDTH * CAMERA_HEIGHT)

// LED
#define LED_ON pi_gpio_pin_write(&gpio_device, 2, 1)
#define LED_OFF pi_gpio_pin_write(&gpio_device, 2, 0)

// streaming
// #define JPEG_STREAMER 1
#define STREAM_WIDTH CAMERA_WIDTH
#define STREAM_HEIGHT CAMERA_HEIGHT

// GAP8 OUTPUT Size
#ifdef REGRESSION_AS_CLASSIFICATION
#define CNN_OUTPUTS 4
#elif IMAV
#define CNN_OUTPUTS 7
#else
#define CNN_OUTPUTS 2
#endif

// Global Variables
static pi_buffer_t buffer;
struct pi_device HyperRam;
static struct pi_device camera;
int32_t data_to_send[CNN_OUTPUTS];

static struct pi_device wifi;
static int open_wifi(struct pi_device *device) {
  struct pi_nina_w10_conf nina_conf;

  pi_nina_w10_conf_init(&nina_conf);

  nina_conf.ssid = "";
  nina_conf.passwd = "";
  nina_conf.ip_addr = "0.0.0.0";
  nina_conf.port = 5555;
  pi_open_from_conf(device, &nina_conf);
  if (pi_transport_open(device))
    return -1;

  return 0;
}

static frame_streamer_t *streamer;
static frame_streamer_t *open_streamer(char *name) {
  struct frame_streamer_conf frame_streamer_conf;

  frame_streamer_conf_init(&frame_streamer_conf);

  frame_streamer_conf.transport = &wifi;
  frame_streamer_conf.format = FRAME_STREAMER_FORMAT_JPEG;
  frame_streamer_conf.width = STREAM_WIDTH;
  frame_streamer_conf.height = STREAM_HEIGHT;
  frame_streamer_conf.depth = 1;
  frame_streamer_conf.name = name;

  return frame_streamer_open(&frame_streamer_conf);
}

static struct pi_device camera;
static void open_camera() {

  int32_t errors = 0;
  uint8_t set_value = 3;
  struct pi_himax_conf cam_conf;

  pi_himax_conf_init(&cam_conf);

  cam_conf.format = PI_CAMERA_QVGA;

  pi_open_from_conf(&camera, &cam_conf);

  errors = pi_camera_open(&camera);

  printf("HiMax camera init:\t\t\t%s\n", errors ? "Failed" : "Ok");

  if (errors)
    pmsis_exit(errors);

  // image rotation
  pi_camera_reg_set(&camera, IMG_ORIENTATION, &set_value);
  pi_camera_control(&camera, PI_CAMERA_CMD_AEG_INIT, 0);
}

void image_crop(uint8_t *image_raw, uint8_t *image_cropped) {
  for (uint16_t i = 0; i < 200; i++) {
    for (uint16_t j = 0; j < 200; j++) {
      *(image_cropped + i * INPUT_WIDTH + j) =
          *(image_raw + (i + 44) * CAMERA_WIDTH + j + 62);
    }
  }
}

// PERFORMANCES
void start_perf_counter() {
#ifdef PERF
  // configure
  pi_perf_conf(1 << PI_PERF_CYCLES);
  // perf measurement begin
  pi_perf_reset();
  pi_perf_start();
#endif
}

void end_perf_counter(bool verbose) {

#ifdef PERF
  // performance measurements: end
  pi_perf_stop();
  float perf_cyc = pi_perf_read(PI_PERF_CYCLES);
  float perf_s = 1. / ((float)perf_cyc / (float)(FREQ_FC * 1000 * 1000));
  // printf("%d\n", perf_cyc);
  printf("%f FPS \n", perf_s);
#endif
}

PI_L2 unsigned char *image_in;
int32_t *ResOut;
/*  ResOut description:
        - Dronet (regression yaw)
                ResOut[0] = steering
                ResOut[1] = collision
        - Dronet (classification yaw)
                ResOut[0] = steering left
                ResOut[1] = straight
                ResOut[2] = steering right
                ResOut[3] = collision
        - Dronet (IMAV)
                ResOut[0] = Edge     visible
                ResOut[0] = Edge not visible
                ResOut[0] = Corner   visible
                ResOut[1] = Yaw
                ResOut[3] = collision left
                ResOut[3] = collision center
                ResOut[3] = collision right
*/

// Checklist
// [x] Set voltage-Freq
// [x] Flash
// [x] open filesystem on flash
// [x] RAM
// [x] UART conf
// [x] Camera
// [x] open cluster
// [x] CNN task
// [x] allocate CNN output tensor
// [x] Network setup
// [x] while 1
void application(void * args) {
  % if l3_supported:
  // Opening of Filesystem and Ram
  mem_init();

  % endif
  // UART Configuration
  struct pi_device uart;
  struct pi_uart_conf conf_uart;
  pi_uart_conf_init(&conf_uart);
  conf_uart.enable_tx = 1;
  conf_uart.enable_rx = 0;
  conf_uart.baudrate_bps = 115200;
  pi_open_from_conf(&uart, &conf_uart);
  if (pi_uart_open(&uart)) {
    printf("Uart open failed !\n");
    pmsis_exit(-1);
  }

  // configure LED
  struct pi_device gpio_device;
  pi_gpio_pin_configure(&gpio_device, 2, PI_GPIO_OUTPUT);
  LED_ON;

  // Open the Himax camera
  open_camera();

  // Initializing network
  ${prefix}network_t network;
  ${prefix}network_initialize(&network);

  // Allocating space for input
  void *input_image_buffer = pi_l2_malloc(${l2_buffer_size});
  if (NULL == input_image_buffer) {
#ifdef VERBOSE
    printf("ERROR: L2 buffer allocation failed.");
#endif
    pmsis_exit(-1);
  }
#ifdef VERBOSE
  printf("\nL2 Buffer alloc initial\t@ 0x%08x:\tOk\n", (unsigned int)input_image_buffer);
#endif

  size_t l2_input_size = ${int(DORY_HW_graph[0].tiling_dimensions["L2"]["input_activation_memory"])};
  printf("Network has been set up\n");

  // Allocate the output tensor
  ResOut = (int32_t *)pi_l2_malloc(CNN_OUTPUTS * sizeof(int32_t));
  if (ResOut == 0) {
    printf("Failed to allocate Memory for Result (%ld bytes)\n",
           CNN_OUTPUTS * sizeof(int32_t));
    return 1;
  }

#ifdef JPEG_STREAMER
  if (open_wifi(&wifi)) {
    printf("Failed to open wifi\n");
    return -1;
  }
  printf("Opened WIFI\n");

  streamer = open_streamer("camera");
  if (streamer == NULL)
    return -1;

  pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, input_image_buffer);
  pi_buffer_set_format(&buffer, STREAM_WIDTH, STREAM_HEIGHT, 1,
                       PI_BUFFER_FORMAT_GRAY);
  printf("Opened streamer\n");
#endif

  // // DEBUG camera
  // uint32_t idx = 0;
  // char ImageName[50];
  // while(1){ // DEBUG CAMERA ACQUISITIONS
  // 	// Start camera acquisition
  // 	sprintf(ImageName, "../../../output_%d.ppm", idx);
  // 	pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
  // 	pi_camera_capture(&camera, input_image_buffer, BUFF_SIZE);
  // 	pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);
  // 	image_crop(input_image_buffer, input_image_buffer);
  // 	WriteImageToFile(ImageName, INPUT_WIDTH, INPUT_HEIGHT,sizeof(uint8_t),
  // input_image_buffer, GRAY_SCALE_IO); 	idx++;
  // }

#ifdef PERF
  float perf_cyc;
  float perf_s;
  pi_perf_conf(1 << PI_PERF_CYCLES);
#endif

  printf("*For* loop...\n");

  for (int iter = 0; iter < 100000; iter++) {
    start_perf_counter();

    LED_OFF;
    // Start camera acquisition
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    pi_camera_capture(&camera, input_image_buffer, BUFF_SIZE);

#ifdef JPEG_STREAMER
    frame_streamer_send(streamer, &buffer);
#endif

// Crop the image
#ifndef IMAV
    image_crop(input_image_buffer, input_image_buffer);
#endif
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    LED_ON;

    // Run CNN inference
    ${prefix}network_args_t args = {
      .l2_buffer = input_image_buffer,
      .l2_buffer_size = ${l2_buffer_size},
      .l2_final_output = ResOut,
      .exec = 0,
      .initial_allocator_dir = 1,
      % if not l3_supported:
      .l2_input_h = ${prefix}L2_input_h
      % endif
    };
    ${prefix}network_run(&network, &args);
    // printf("main.c: Steering Angle: %d, Collision: %d \n",  ResOut[0],
    // ResOut[1]);

    // prepare data for UART send
    for (int i = 0; i < CNN_OUTPUTS; i++) {
      data_to_send[i] = ResOut[i];
    }

    /* UART synchronous send */
    // pi_uart_write(&uart, (char *) data_to_send, CNN_OUTPUTS*4);

    /* UART asynchronous send */
    pi_task_t wait_task2 = {0};
    pi_task_block(&wait_task2);
    pi_uart_write_async(&uart, (char *)data_to_send, CNN_OUTPUTS * 4,
                        &wait_task2);
    // pi_task_wait_on(&wait_task2);

    end_perf_counter(true);
  }

  ${prefix}network_terminate(&network);
  pi_l2_free(input_image_buffer, ${l2_buffer_size});
  pmsis_exit(0);
}

int main () {
  PMU_set_voltage(${soc_voltage_mv}, 0);
  pi_time_wait_us(10000);
  pi_freq_set(PI_FREQ_DOMAIN_FC, ${fc_frequency});
  pi_time_wait_us(10000);
  pi_freq_set(PI_FREQ_DOMAIN_CL, ${cl_frequency});
  pi_time_wait_us(10000);

  #ifdef VERBOSE
  printf("Set VDD voltage as %.2f, FC Frequency as %d MHz, CL Frequency = %d MHz\n",
         (float)${soc_voltage_mv} / 1000, ${fc_frequency} / 1000000, ${cl_frequency} / 1000000);
  #endif

  pmsis_kickoff((void*)application);
  return 0;
}
