/*
 * Copyright (C) 2023 University of Bologna, Italy, ETH Zurich Switzerland.
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * See LICENSE.apache.md in the top directory for details.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * File:    main.c
 * Author:  Lorenzo Lamberti      <lorenzo.lamberti@unibo.it>
 *          Luka Macan            <luka.macan@unibo.it>
 * Date:    21.09.2023
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

// --- custom ---
#include "himax_registers.h"
#include "himax_utils.h"

// DEBUG PRINTS
// #define VERBOSE 1
// #define PERF 1 // print FPS performances of the network

/* OTHER DEBUG FLAGS */
// #define LOAD_CHECKSUM_INPUT 1

// Defines
#define FREQ_FC 200
#define FREQ_CL 100

// Camera
#define CAMERA_WIDTH 162
#define CAMERA_HEIGHT 122
#define INPUT_WIDTH 162
#define INPUT_HEIGHT 122
#define INPUT_COLORS 1

#define CAMERA_SIZE (CAMERA_HEIGHT * CAMERA_WIDTH)
#define BUFF_SIZE (CAMERA_WIDTH * CAMERA_HEIGHT)

// LED
static struct pi_device gpio_device;
#define LED_ON pi_gpio_pin_write(&gpio_device, 2, 1)
#define LED_OFF pi_gpio_pin_write(&gpio_device, 2, 0)
uint8_t led = 0;
#define LED_TOGGLE (led ^= 1, pi_gpio_pin_write(&gpio_device, 2, led))

// streaming
// #define JPEG_STREAMER 1
#define STREAM_WIDTH CAMERA_WIDTH
#define STREAM_HEIGHT CAMERA_HEIGHT

// GAP8 OUTPUT Size
#ifdef REGRESSION_AS_CLASSIFICATION
#define CNN_OUTPUTS 4
#else
#define CNN_OUTPUTS 2
#endif

// Global Variables
static pi_buffer_t buffer_streamer[2];
static struct pi_device camera;
static int32_t data_to_send[CNN_OUTPUTS];

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

void manual_exposure_calibration() {
  uint8_t * tmp_image_buffer = (uint8_t *)pmsis_l2_malloc(CAMERA_SIZE);
  if (NULL == tmp_image_buffer) {
    printf("ERROR: Out of memory. Cannot allocate tmp_image_buffer.\n");
    pmsis_exit(-1);
  }

  const unsigned int exposure_calibration_size = 10;
  init_exposure_calibration(exposure_calibration_size);
  printf("HiMax Exposure init\n");
  
  for (int i = 0; i < exposure_calibration_size; i++) {
    // Start camera acquisition
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    pi_camera_capture(&camera, tmp_image_buffer, BUFF_SIZE);
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

    himax_update_exposure(&camera);
  }

  pmsis_l2_malloc_free(tmp_image_buffer, CAMERA_SIZE);
}

void manual_exposure() {
  // Dronet parameters: to be changed !
  uint16_t integration_value16 = 0x0154;
  uint16_t d_gain_value16 = 0x0128;
  uint8_t a_gain_value = 0x00;
  write_himax_exp_params(&camera, integration_value16, d_gain_value16,
                         a_gain_value);
}

static struct pi_device camera;
static void open_camera() {

  int32_t errors = 0;
  uint8_t set_value = 3;
  struct pi_himax_conf cam_conf;

  pi_himax_conf_init(&cam_conf);

  cam_conf.format = PI_CAMERA_QQVGA;

  pi_open_from_conf(&camera, &cam_conf);

  errors = pi_camera_open(&camera);

  printf("HiMax camera init:\t\t\t%s\n", errors ? "Failed" : "Ok");

  if (errors)
    pmsis_exit(errors);

  // --- Start procedure for writing the registers
  set_value = 0x00; // consume
  pi_camera_reg_set(&camera, HIMAX_GRP_PARAM_HOLD, &set_value);
  pi_time_wait_us(100000);

  // --- image format
  himax_set_format(&camera, QQVGA);

  // --- image FPS
  himax_set_fps(&camera, 60, QQVGA);

  // image rotation
  set_value = 3;
  pi_camera_reg_set(&camera, IMG_ORIENTATION, &set_value);
  pi_camera_control(&camera, PI_CAMERA_CMD_AEG_INIT, 0);

  // --- End procedure for writing the registers (commit)
  set_value = 0x01; // hold
  pi_camera_reg_set(&camera, HIMAX_GRP_PARAM_HOLD, &set_value);
  pi_time_wait_us(100000);
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

// CNN POST-PROCESSING
#ifdef DEBUG
static inline float sigmoid(float x) {
  return 1. / (1. + expf(-x));
}
#endif


// ==========================================================================
// ==============================   MAIN   ==================================
// ==========================================================================

void application(void * args) {

  // Voltage and Frequency settings
  uint32_t voltage = 1200;
  PMU_set_voltage(voltage, 0);
  pi_time_wait_us(10000);
  pi_freq_set(PI_FREQ_DOMAIN_FC, FREQ_FC * 1000 * 1000);
  pi_time_wait_us(10000);
  pi_freq_set(PI_FREQ_DOMAIN_CL, FREQ_CL * 1000 * 1000);
  pi_time_wait_us(10000);
  printf("Set VDD voltage as %.2f, FC Frequency as %d MHz, CL Frequency = %d "
         "MHz\n",
         (float)voltage / 1000, FREQ_FC, FREQ_CL);

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
  pi_task_t task_uart = {0};
  pi_task_block(&task_uart);
  printf("Uart opened !\n");

  // configure LED
  pi_gpio_pin_configure(&gpio_device, 2, PI_GPIO_OUTPUT);
  LED_ON;

  // Initializing network
  ${prefix}network_t network;
  ${prefix}network_initialize(&network);

  // Allocating space the network and inputs
<%
l2_input_size = int(DORY_HW_graph[0].tiling_dimensions["L2"]["input_activation_memory"])
%>\
  const size_t network_l2_buffer_size = ${l2_buffer_size};
  const size_t network_l2_input_size = ${l2_input_size};
  // Total size is network (which contains already space for 1 input) + one more input
  // for double buffering.
  const size_t total_l2_size = network_l2_buffer_size + network_l2_input_size;

  void *l2_buffer = pmsis_l2_malloc(total_l2_size);
  if (NULL == l2_buffer) {
#ifdef VERBOSE
    printf("ERROR: L2 buffer allocation failed.");
#endif
    pmsis_exit(-1);
  }
#ifdef VERBOSE
  printf("\nL2 Buffer alloc initial\t@ 0x%08x:\tOk\n", (unsigned int)l2_buffer);
#endif

  /* Double buffer */
  const int N_IMAGE_BUFFERS = 2;
  const void *input_addr[2] = { l2_buffer, l2_buffer + network_l2_buffer_size };
  const void *network_start_addr[2] = { l2_buffer, l2_buffer + network_l2_input_size };
  int buffer_idx = 0;

#ifdef LOAD_CHECKSUM_INPUT
  size_t input_size = 1000000;
  void *ram_input = ram_malloc(input_size);
  load_file_to_ram(ram_input, "inputs.hex");
#endif

  printf("Network has been set up\n");

  // Open the Himax camera
  open_camera();

#ifdef JPEG_STREAMER
  if (open_wifi(&wifi)) {
    printf("Failed to open wifi\n");
    return -1;
  }
  printf("Opened WIFI\n");

  streamer = open_streamer("camera");
  if (streamer == NULL)
    return -1;

  for (int i = 0; i < N_IMAGE_BUFFERS; i++) {
    pi_buffer_init(&buffer_streamer[i], PI_BUFFER_TYPE_L2, input_addr[i]);
    pi_buffer_set_format(&buffer_streamer[i], STREAM_WIDTH, STREAM_HEIGHT, 1,
                         PI_BUFFER_FORMAT_GRAY);
  }

  printf("Opened streamer\n");
#endif

#ifdef PERF
  pi_perf_conf(1 << PI_PERF_CYCLES);
#endif

// ==========================================================================
// =======================   First camera fetch   ===========================
// ==========================================================================

    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    pi_camera_capture(&camera, input_addr[buffer_idx], BUFF_SIZE);
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

#ifdef LOAD_CHECKSUM_INPUT
    ram_read(input_addr[buffer_idx], ram_input, network_l2_input_size);
#endif

// ==========================================================================
// ==============================   Loop   ==================================
// ==========================================================================

  printf("*For* loop...\n");

  for (int iter = 0; iter < 100000; iter++) {
    start_perf_counter();

    LED_TOGGLE;

    // Next image fetch

    pi_task_t camera_task;
    pi_camera_control(&camera, PI_CAMERA_CMD_START, 0);
    pi_camera_capture_async(&camera, input_addr[!buffer_idx], BUFF_SIZE, pi_task_block(&camera_task));

#ifdef LOAD_CHECKSUM_INPUT
    // TODO: Make it async too
    ram_read(input_addr[!buffer_idx], ram_input, network_l2_input_size);
#endif

    ${prefix}network_args_t network_args = {
      .l2_buffer = network_start_addr[buffer_idx],
      .l2_buffer_size = network_l2_buffer_size,
      .l2_final_output = data_to_send,
      .exec = 0,
      .initial_allocator_dir = !buffer_idx, // opposite of buffer_idx
      % if not l3_supported:
      .l2_input_h = ${prefix}L2_input_h
      % endif
    };

    ${prefix}network_run_async(&network, &network_args);
    ${prefix}network_run_wait(&network);

    // Print CNN outputs: Steering and collision
#ifdef DEBUG
    int32_t angle = data_to_send[0];
    int32_t prob_of_col = data_to_send[1];
    // printf("network.c: Steering Angle: %d, Collision: %d \n",  angle,
    // prob_of_col);

    // de-quantize and convert to float --> only for debugging ! this is done by
    // the STM32
    float quantum = 0.0006;
    float angle_float = (float)angle * quantum;
    float prob_of_col_float = sigmoid((float)prob_of_col * quantum);
    printf("network.c: Steering Angle: %.2f, Collision: %.2f \n", angle_float,
           prob_of_col_float);
#endif

    /* UART synchronous send */
    // pi_uart_write(&uart, (char *) data_to_send, CNN_OUTPUTS*4);

    /* UART asynchronous send */
    pi_uart_write_async(&uart, (char *)data_to_send, CNN_OUTPUTS * 4,
                        pi_task_block(&task_uart));
    // pi_task_wait_on(&task_uart);

    // Wait for image

    pi_task_wait_on(&camera_task);
    pi_camera_control(&camera, PI_CAMERA_CMD_STOP, 0);

#ifdef JPEG_STREAMER
    frame_streamer_send(streamer, &buffer_streamer[buffer_idx]);
#endif

    // Update buffer
    buffer_idx = !buffer_idx;

    end_perf_counter(true);
  }

#ifdef LOAD_CHECKSUM_INPUTS
  ram_free(ram_input, input_size);
#endif
  ${prefix}network_terminate(&network);
  pmsis_l2_malloc_free(l2_buffer, total_l2_size);
}

int main () {
  pmsis_kickoff((void*)application);
  return 0;
}
