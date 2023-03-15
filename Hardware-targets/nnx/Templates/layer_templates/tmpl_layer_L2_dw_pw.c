/*
 * layer_template_nnx.c
 * Francesco Conti <f.conti@unibo.it>
 * Alessio Burrello <alessio.burrello@unibo.it>
 * Luka Macan <luka.macan@unibo.it>
 *
 * Copyright (C) 2018-2022 University of Bologna
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

#include "${func_name}.h"
#include "pulp_nnx.h"
#include "pulp_nnx_util.h"
#include "network.h"
#include "dory_dma_v2.h"
#include "dory_get_tile.h"
#include "layer_debug.h"
#include "tile_status.h"
#include "execute.h"

static const Kernel kernel_dw = {
    .shape = {
        .height = ${fs1},
        .width = ${fs2}
    },
    .stride = {
        .height = ${s[0]},
        .width = ${s[1]}
    },
    .groups = ${g}
};

static const Kernel kernel_pw = {
    .shape = {
        .height = 1,
        .width = 1
    },
    .stride = {
        .height = 1,
        .width = 1
    },
    .groups = 1
};

static const TileIndex end_index = {
    .height = ${tile_dim_h},
    .width = ${tile_dim_w},
    .output_channel = ${tile_dim_nof}
};

static const Layer body_dw = {
    .input = {
        .height = ${x_tile_size_h},
        .width = ${x_tile_size_w},
        .channel = ${x_tile_size_nif}
    },
    .output = {
        .height = ${y_tile_size_h},
        .width = ${y_tile_size_w},
        .channel = ${x_tile_size_nif}
    }
};

static const Layer body_pw = {
    .input = {
        .height = ${y_tile_size_h},
        .width = ${y_tile_size_w},
        .channel = ${x_tile_size_nif}
    },
    .output = {
        .height = ${y_tile_size_h},
        .width = ${y_tile_size_w},
        .channel = ${y_tile_size_nof}
    }
};

static const Layer border_dw = {
    .input = {
        .height = ${x_tile_size_h_last},
        .width = ${x_tile_size_w_last},
        .channel = ${x_tile_size_nif_last}
    },
    .output = {
        .height = ${y_tile_size_h_last},
        .width = ${y_tile_size_w_last},
        .channel = ${x_tile_size_nif_last}
    }
};

static const Layer border_pw = {
    .input = {
        .height = ${y_tile_size_h_last},
        .width = ${y_tile_size_w_last},
        .channel = ${x_tile_size_nif_last}
    },
    .output = {
        .height = ${y_tile_size_h_last},
        .width = ${y_tile_size_w_last},
        .channel = ${y_tile_size_nof_last}
    }
};

static void load_input_prepare(Layer tile, Layer body, Layer layer, TileIndex index, DmaTransferConf * const conf) {
    // additionally overlap by padding for the first tile after a border one
    // this because in the first tile we use less pixels from x_buffer, since we have the ones of padding
    const int x_offset_h = index.height > 0 ? layer.padding.top : 0;
    const int x_offset_w = index.width > 0 ? layer.padding.left : 0;

    conf->ext = dory_get_tile_3d(layer.addr.input,
                                   index.height, index.width, 0,
                                   body.input.height, body.input.width, body.input.channel,
                                   ${x_w}, ${nif*g},
                                   ${conv_overlap1}, ${conv_overlap2}, 0,
                                   x_offset_h, x_offset_w, 0,
                                   ${x_data_size_byte});
    conf->loc = tile.addr.input;
    conf->number_of_2d_copies = tile.input.height;
    conf->number_of_1d_copies = tile.input.width;
    conf->length_1d_copy = tile.input.channel;
    conf->stride_2d = ${l1_x_dma_stride_2d};
    conf->stride_1d = ${l1_x_dma_stride_1d};
    conf->dir = 1;
}

static void load_weights_prepare(Layer tile, Kernel kernel,
                                 MemoryStatus status_weights,
                                 MemoryStatus status_scale,
                                 MemoryStatus status_bias,
                                 DmaTransferConf * const conf_weights,
                                 DmaTransferConf * const conf_scale,
                                 DmaTransferConf * const conf_bias
                                 ) {
    // Special because of accelerators special memory layout
    const int size_weights = (kernel.groups > 1 ? DIVNCEIL(tile.output.channel, 16) : tile.output.channel) * ${l1_W_tile_ki_size};
    const int size_scale = tile.output.channel * ${int(act_dim_bit/8)};
    const int size_bias = tile.output.channel * ${int(bias_bits/8)};

    #define CONF_SET(name)                                     ${"\\"}
        conf_ ## name->ext = status_ ## name.addr_ext; ${"\\"}
        conf_ ## name->loc = tile.addr.name;           ${"\\"}
        conf_ ## name->length_1d_copy = size_ ## name;         ${"\\"}
        conf_ ## name->dir = 1;

    CONF_SET(weights);
    CONF_SET(scale);
    CONF_SET(bias);
}

static void store_prepare(Layer tile, Layer body, Layer layer, TileIndex index, DmaTransferConf * const conf) {
    conf->ext = dory_get_tile_3d(layer.addr.output,
                                         index.height, index.width, index.output_channel,
                                         body.output.height, body.output.width, body.output.channel,
                                         ${y_w}, ${int(nof*factor)},
                                         0, 0, 0,
                                         0, 0, 0,
                                         ${y_data_size_byte});
    conf->loc = tile.addr.output;
    conf->number_of_2d_copies = tile.output.height;
    conf->number_of_1d_copies = tile.output.width;
    conf->length_1d_copy = tile.output.channel;
    conf->stride_2d = ${l1_y_dma_stride_2d};
    conf->stride_1d = ${l1_y_dma_stride_1d};
    conf->dir = 0;
}

static void load_input_async(Layer tile, TileStatus status, Layer body, Layer layer) {
    if (status->input.is_transfer) {
        DmaTransferConf conf_input;
        load_input_prepare(tile, body, layer, status->index, &conf_input);
        dma_transfer_async(conf_input);
    }
}

static void load_weights_async(Layer tile, TileStatus * const status, Kernel kernel) {
    if (status->weights.is_transfer) {
        DmaTransferConf conf_weights, conf_scale, conf_bias;

        load_weights_prepare(tile, kernel,
                             status->weights,
                             status->scale,
                             status->bias,
                             &conf_weights,
                             &conf_scale,
                             &conf_bias);
        dma_transfer_1d_async(conf_weights);
        dma_transfer_1d_async(conf_scale);
        dma_transfer_1d_async(conf_bias);

        status->weights.addr_ext += conf_weights.length_1d_copy;
        status->scale.addr_ext += conf_scale.length_1d_copy;
        status->bias.addr_ext += conf_bias.length_1d_copy;
    }
}

void ${func_name}(void *args) {

    #ifdef DEBUG_GVSOC
    nnx_activate_gvsoc_logging(GVSOC_LOGGING_FORMAT_DECIMAL);
    #endif

    layer_args_t *layer_args = (layer_args_t *)args;

    // Initialization

    // TODO offsets
    
    Layer layer_dw = {
        .addr = {
            .input = layer_args->L2_input,
            .weights = layer_args->L2_weights,
            .scale = layer_args->L2_weights + ${l2_k_offset},
            .bias = layer_args->L2_weights + ${l2_lambda_offset},
            .output = layer_args->L2_output
        },
        .padding = {
            .top    = layer_args->padding & PAD_TOP ? ${padding_top} : DONT_PAD,
            .right  = ${padding_right},
            .bottom = layer_args->padding & PAD_BOTTOM ? ${padding_bottom} : DONT_PAD,
            .left   = ${padding_left}
        }
    };

    Layer layer_pw = {
        .addr = {
            .input = layer_args->L2_input,
            .weights = layer_args->L2_weights,
            .scale = layer_args->L2_weights + ${l2_k_offset},
            .bias = layer_args->L2_weights + ${l2_lambda_offset},
            .output = layer_args->L2_output
        },
        .padding = {
            .top    = layer_args->padding & PAD_TOP ? ${padding_top} : DONT_PAD,
            .right  = ${padding_right},
            .bottom = layer_args->padding & PAD_BOTTOM ? ${padding_bottom} : DONT_PAD,
            .left   = ${padding_left}
        }
    };

    // Double buffer address init

    // TODO offsets

    const unsigned int l1_buffer = layer_args->L1_buffer;
    const int l1_buffer_input = l1_buffer + ${l1_x_offset};
    const int l1_buffer_output = l1_buffer + ${l1_y_offset};
    const int l1_buffer_weights = l1_buffer + ${l1_W_offset};
    const int l1_buffer_scale = l1_buffer + ${l1_k_offset};
    const int l1_buffer_bias = l1_buffer + ${l1_lambda_offset};

    Address buffer_addresses_dw[2] = {
        {
            .input = l1_buffer_input,
            .weights = l1_buffer_weights,
            .scale = l1_buffer_scale,
            .bias = l1_buffer_bias,
            .output = l1_buffer_output
        },
        {
            .input = l1_buffer_input + ${l1_x_tile_size},
            .weights = l1_buffer_weights + ${l1_W_tile_size},
            .scale = l1_buffer_scale + ${l1_k_tile_size},
            .bias = l1_buffer_bias + ${l1_lambda_tile_size},
            .output = l1_buffer_output + ${l1_y_tile_size}
        }
    };

    Address buffer_addresses_pw[2] = {
        {
            .input = l1_buffer_input,
            .weights = l1_buffer_weights,
            .scale = l1_buffer_scale,
            .bias = l1_buffer_bias,
            .output = l1_buffer_output
        },
        {
            .input = l1_buffer_input + ${l1_x_tile_size},
            .weights = l1_buffer_weights + ${l1_W_tile_size},
            .scale = l1_buffer_scale + ${l1_k_tile_size},
            .bias = l1_buffer_bias + ${l1_lambda_tile_size},
            .output = l1_buffer_output + ${l1_y_tile_size}
        }
    };

    // Init nnx tasks
    nnx_task_t nnx_task = nnx_task_create(
            ${fs1}, ${int(flag_DW)},
            ${x_data_size_byte}, ${y_data_size_byte}, ${W_data_size},
            weightOffsetModeLayerWise, ${-(2**(W_data_size-1))},
            (nnx_quant_t) {
                .shift_amount = layer_args->out_shift,
                .mode = quantMode8Bit,
                .function = quantFunctionRelu,
                .flag_rounding = FLAG_UNUSED
            }, (nnx_norm_t) {
                .mode  = ${"normMode32Bit" if act_dim_bit == 32 else "normMode8Bit" if act_dim_bit == 8 else "FLAG_UNUSED"},
                .flag_bias  = FLAG_USED,
                .flag_shift = FLAG_UNUSED
            }, ${stride});

    nnx_task_t nnx_task = nnx_task_create(
            1, 0,
            ${x_data_size_byte}, ${y_data_size_byte}, ${W_data_size},
            weightOffsetModeLayerWise, ${-(2**(W_data_size-1))},
            (nnx_quant_t) {
                .shift_amount = layer_args->out_shift,
                .mode = quantMode8Bit,
                .function = quantFunctionRelu,
                .flag_rounding = FLAG_UNUSED
            }, (nnx_norm_t) {
                .mode  = ${"normMode32Bit" if act_dim_bit == 32 else "normMode8Bit" if act_dim_bit == 8 else "FLAG_UNUSED"},
                .flag_bias  = FLAG_USED,
                .flag_shift = FLAG_UNUSED
            }, 1);

    nnx_init();

    #define MEMORY_STATUS_INIT(layer, name) ${"\\"}
        .name = {                           ${"\\"}
            .addr_ext = layer.addr.name,    ${"\\"}
            .is_transfer = 1,               ${"\\"}
            .buffer_index = 0               ${"\\"}
        }

    TileStatus tile_status_dw = {
        .index = { 0 },
        MEMORY_STATUS_INIT(layer_dw, input),
        MEMORY_STATUS_INIT(layer_dw, weights),
        MEMORY_STATUS_INIT(layer_dw, scale),
        MEMORY_STATUS_INIT(layer_dw, bias),
        MEMORY_STATUS_INIT(layer_dw, output)
    };

    TileStatus tile_status_pw = {
        .index = { 0 },
        MEMORY_STATUS_INIT(layer_pw, input),
        MEMORY_STATUS_INIT(layer_pw, weights),
        MEMORY_STATUS_INIT(layer_pw, scale),
        MEMORY_STATUS_INIT(layer_pw, bias),
        MEMORY_STATUS_INIT(layer_pw, output)
    };

    const int total_tiles = end_index.height * end_index.width * end_index.output_channel;

    for (int i_tile = 0; i_tile < total_tiles; i_tile++) {
        Layer tile_dw = tile_create(tile_status_dw.index, end_index, body_dw, border_dw, layer_dw,
                                 tile_status_get_addr(tile_status_dw, buffer_addresses_dw));
        Layer tile_pw = tile_create(tile_status_pw.index, end_index, body_pw, border_pw, layer_pw,
                                 tile_status_get_addr(tile_status_pw, buffer_addresses_pw));

        DmaTransfer transfer = dma_transfer_create();
        load_input_async(tile_dw, tile_status_dw, body_dw, layer_dw);
        load_weights_async(tile_dw, &tile_status_dw, kernel_dw);
        load_weights_async(tile_pw, &tile_status_pw, kernel_pw);
        dma_transfer_wait(transfer);

        execute_prepare(tile_dw, &nnx_task_dw);
        execute_wait(execute_async(nnx_task_dw));

        execute_prepare(tile_pw, &nnx_task_pw);
        execute_wait(execute_async(nnx_task_pw));

        DmaTransfer transfer_store = dma_transfer_create();
        DmaTransferConf store_conf;
        store_prepare(tile_pw, body_pw, layer_pw, tile_status_pw.index, &store_conf);
        dma_transfer_async(store_conf);
        dma_transfer_wait(transfer_store);

        tile_status_dw = tile_status_get_next(tile_status_dw, end_index);
        tile_status_pw = tile_status_get_next(tile_status_pw, end_index);
    }


    // Terminate

    nnx_term();
}
