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

% if verbose:
#define VERBOSE 1
% endif

% if sdk == 'pulp-sdk':
unsigned int PMU_set_voltage(unsigned int Voltage, unsigned int CheckFrequencies) {
  return 0;
}
% endif


void application(void * arg) {
  % if l3_supported:
  // Opening of Filesystem and Ram
  mem_init();

  % endif
  // Initializing network
  ${prefix}network_t network;
  ${prefix}network_initialize(&network);

  // led init
  struct pi_device gpio_device;
  pi_gpio_pin_configure(&gpio_device, 2, PI_GPIO_OUTPUT);
  int led = 1;

  // Allocating space for input
  void *l2_buffer = pi_l2_malloc(${l2_buffer_size});
  if (NULL == l2_buffer) {
#ifdef VERBOSE
    printf("ERROR: L2 buffer allocation failed.");
#endif
    pmsis_exit(-1);
  }
#ifdef VERBOSE
  printf("\nL2 Buffer alloc initial\t@ 0x%08x:\tOk\n", (unsigned int)l2_buffer);
#endif

  size_t l2_input_size = ${int(DORY_HW_graph[0].tiling_dimensions["L2"]["input_activation_memory"])};
  size_t input_size = 1000000;

  % if l3_supported:
  void *ram_input = ram_malloc(input_size);
  load_file_to_ram(ram_input, "${prefix}inputs.hex");
  % endif

  for (int iter = 0; iter < 1000; iter++) {
    pi_gpio_pin_write(&gpio_device, 2, led);
    led ^= 1;
    % if l3_supported:
    ram_read(l2_buffer, ram_input, l2_input_size);
    % endif
    ${prefix}network_args_t args = {
      .l2_buffer = l2_buffer,
      .l2_buffer_size = ${l2_buffer_size},
      .l2_final_output = l2_buffer,
      .exec = 0,
      .initial_allocator_dir = 1,
      % if not l3_supported:
      .l2_input_h = ${prefix}L2_input_h
      % endif
    };
    ${prefix}network_run(&network, &args);
  }

  % if l3_supported:
  ram_free(ram_input, input_size);
  % endif
  ${prefix}network_terminate(&network);
  pi_l2_free(l2_buffer, ${l2_buffer_size});
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
