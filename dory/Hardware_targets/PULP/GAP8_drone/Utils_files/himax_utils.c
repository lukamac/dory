/**
 * File for AI SAIL (camera streaming via pipe/wifi from the AI-DECK)
 * https://github.com/jeguzzi/ai_sail
 *
 * Author: Jérôme Guzzi (jerome@idsia.ch)
 * License: MIT
 */

#include "pmsis.h"
#include "bsp/camera.h"
#include "bsp/camera/himax.h"
#include "himax_utils.h"

// Apparently binning cannot be changed once the camera is running.
// unless AE is on!
// [QQVGA] window is fine instead

#define         HIMAX_MODEL_ID_H          0x0000
#define         HIMAX_MODEL_ID_L          0x0001
#define         HIMAX_FRAME_COUNT         0x0005
#define         HIMAX_PIXEL_ORDER         0x0006
// R&W registers
// Sensor mode control
#define         HIMAX_MODE_SELECT         0x0100
#define         HIMAX_IMG_ORIENTATION     0x0101
#define         HIMAX_SW_RESET            0x0103
#define         HIMAX_GRP_PARAM_HOLD      0x0104
// Sensor exposure gain control
#define         HIMAX_INTEGRATION_H       0x0202
#define         HIMAX_INTEGRATION_L       0x0203
#define         HIMAX_ANAprintf_GAIN         0x0205
#define         HIMAX_DIGITAL_GAIN_H      0x020E
#define         HIMAX_DIGITAL_GAIN_L      0x020F
// Frame timing control
#define         HIMAX_FRAME_LEN_LINES_H   0x0340
#define         HIMAX_FRAME_LEN_LINES_L   0x0341
#define         HIMAX_LINE_LEN_PCK_H      0x0342
#define         HIMAX_LINE_LEN_PCK_L      0x0343
// Binning mode control
#define         HIMAX_READOUT_X           0x0383
#define         HIMAX_READOUT_Y           0x0387
#define         HIMAX_BINNING_MODE        0x0390
// Test pattern control
#define         HIMAX_TEST_PATTERN_MODE   0x0601
// Black level control
#define         HIMAX_BLC_CFG             0x1000
#define         HIMAX_BLC_TGT             0x1003
#define         HIMAX_BLI_EN              0x1006
#define         HIMAX_BLC2_TGT            0x1007
//  Sensor reserved
#define         HIMAX_DPC_CTRL            0x1008
#define         HIMAX_SINGLE_THR_HOT      0x100B
#define         HIMAX_SINGLE_THR_COLD     0x100C
// VSYNC,HSYNC and pixel shift register
#define         HIMAX_VSYNC_HSYNC_PIXEL_SHIFT_EN  0x1012
// Automatic exposure gain control
#define         HIMAX_AE_CTRL             0x2100
#define         HIMAX_AE_TARGET_MEAN      0x2101
#define         HIMAX_AE_MIN_MEAN         0x2102
#define         HIMAX_CONVERGE_IN_TH      0x2103
#define         HIMAX_CONVERGE_OUT_TH     0x2104
#define         HIMAX_MAX_INTG_H          0x2105
#define         HIMAX_MAX_INTG_L          0x2106
#define         HIMAX_MIN_INTG            0x2107
#define         HIMAX_MAX_AGAIN_FULL      0x2108
#define         HIMAX_MAX_AGAIN_BIN2      0x2109
#define         HIMAX_MIN_AGAIN           0x210A
#define         HIMAX_MAX_DGAIN           0x210B
#define         HIMAX_MIN_DGAIN           0x210C
#define         HIMAX_DAMPING_FACTOR      0x210D
#define         HIMAX_FS_CTRL             0x210E
#define         HIMAX_FS_60HZ_H           0x210F
#define         HIMAX_FS_60HZ_L           0x2110
#define         HIMAX_FS_50HZ_H           0x2111
#define         HIMAX_FS_50HZ_L           0x2112
#define         HIMAX_FS_HYST_TH          0x2113
// Motion detection control
#define         HIMAX_MD_CTRL             0x2150
#define         HIMAX_I2C_CLEAR           0x2153
#define         HIMAX_WMEAN_DIFF_TH_H     0x2155
#define         HIMAX_WMEAN_DIFF_TH_M     0x2156
#define         HIMAX_WMEAN_DIFF_TH_L     0x2157
#define         HIMAX_MD_THH              0x2158
#define         HIMAX_MD_THM1             0x2159
#define         HIMAX_MD_THM2             0x215A
#define         HIMAX_MD_THL              0x215B
//  Sensor timing control
#define         HIMAX_QVGA_WIN_EN         0x3010
#define         HIMAX_SIX_BIT_MODE_EN     0x3011
#define         HIMAX_PMU_AUTOSLEEP_FRAMECNT  0x3020
#define         HIMAX_ADVANCE_VSYNC       0x3022
#define         HIMAX_ADVANCE_HSYNC       0x3023
#define         HIMAX_EARLY_GAIN          0x3035
//  IO and clock control
#define         HIMAX_BIT_CONTROL         0x3059
#define         HIMAX_OSC_CLK_DIV         0x3060
#define         HIMAX_ANA_Register_11     0x3061
#define         HIMAX_IO_DRIVE_STR        0x3062
#define         HIMAX_IO_DRIVE_STR2       0x3063
#define         HIMAX_ANA_Register_14     0x3064
#define         HIMAX_OUTPUT_PIN_STATUS_CONTROL   0x3065
#define         HIMAX_ANA_Register_17     0x3067
#define         HIMAX_PCLK_POLARITY       0x3068

static void reset_exposure_calibration();
void init_exposure_calibration(unsigned int calibration_size);

int himax_open(struct pi_device *camera)
{
  struct pi_himax_conf cam_conf;
  pi_himax_conf_init(&cam_conf);

  cam_conf.format = PI_CAMERA_QVGA;

  pi_open_from_conf(camera, &cam_conf);

  if (pi_camera_open(camera))
  {
    printf("Failed to open Himax camera\n");
    return -1;
  }
  return 0;
}

void himax_close(struct pi_device *camera)
{
  pi_camera_close(camera);
}

static void _himax_enable_ae(struct pi_device *camera, uint8_t value)
{
  if(value) value=1;
  pi_camera_reg_set(camera, HIMAX_AE_CTRL, &value);
}

void himax_enable_ae(struct pi_device *camera, uint8_t value)
{
  _himax_enable_ae(camera, value);
  if(!value) reset_exposure_calibration();
}

static int32_t pi_camera_reg_get16(struct pi_device *camera, uint32_t reg_addr_l, uint32_t reg_addr_h, uint16_t *value)
{
  uint8_t *v = (uint8_t *) value;
  pi_camera_reg_get(camera, reg_addr_l, v);
  return pi_camera_reg_get(camera, reg_addr_h, v+1);
}

static int32_t pi_camera_reg_set16(struct pi_device *camera, uint32_t reg_addr_l, uint32_t reg_addr_h, uint16_t *value)
{
  uint8_t *v = (uint8_t *) value;
  pi_camera_reg_set(camera, reg_addr_l, v);
  return pi_camera_reg_set(camera, reg_addr_h, v+1);
}

static int min_line_length(uint8_t format){
  switch (format) {
    case FULL: case QVGA:
      return 0x0178; // 376
    case HALF: case QQVGA:
      return 0x00D7;// 215
    default:
      return 0x0178; // 376
  }
}

static int min_number_of_lines(uint8_t format){
  switch (format) {
    case FULL:
      return 0x0158; // 344
    case QVGA:
      return 0x0104; //260
    // This is not defined in the datasheet, so we are guessing
    // that is proportional to the other values
    case HALF:
      return 170;
    case QQVGA:
      return 0x0080;
    default:
      return 0x0158;
  }
}

// TODO: Default 6Mhz, but we should read it from the registers.
#define FREQ 6000000

static int max_fps(uint8_t format)
{
  /*
  QVGA: 61fps
  */
  return FREQ / min_line_length(format) / min_number_of_lines(format);
}

static int set_fps(uint8_t format)
{
  /*
  QVGA: fix line length pck
  calcola frame len
  */
  return FREQ / min_line_length(format) / min_number_of_lines(format);
}

uint8_t himax_set_fps(struct pi_device *camera, uint8_t fps, uint8_t format)
{
    int m_fps = max_fps(format);
    if(fps > m_fps){
      printf("Desired fps %d is too high, will cap it to %d\n", fps, m_fps);
      fps = m_fps;
    }
    int line_length = min_line_length(format) + 1;
    int number_of_lines = FREQ / line_length / fps;
    uint8_t value;
    value = (uint8_t) (number_of_lines >> 8);
    pi_camera_reg_set(camera, HIMAX_FRAME_LEN_LINES_H, &value);
    value = (uint8_t) (0xFF & number_of_lines);
    pi_camera_reg_set(camera, HIMAX_FRAME_LEN_LINES_L, &value);
    value = (uint8_t) (line_length >> 8);
    pi_camera_reg_set(camera, HIMAX_LINE_LEN_PCK_H, &value);
    value = (uint8_t) (0xFF & line_length);
    pi_camera_reg_set(camera, HIMAX_LINE_LEN_PCK_L, &value);
    printf("Set fps to %d (%d x %d @ %d MHz)\n", fps, line_length, number_of_lines, FREQ / 1000000);

    printf("frame-rate:\t\t%d\n", (FREQ/1)/(line_length*number_of_lines));

    reset_exposure_calibration();
    return fps;
}


void himax_set_format(struct pi_device *camera, uint8_t format)
{
  uint8_t value;
  if(format == QQVGA || format == HALF)
  {
    value=0x03;
  }
  else
  {
    value = 0x01;
  }
  pi_camera_reg_set(camera, HIMAX_READOUT_X, &value);
  pi_camera_reg_set(camera, HIMAX_READOUT_Y, &value);
  pi_camera_reg_set(camera, HIMAX_BINNING_MODE, &value);

  if(format == QQVGA || format == QVGA)
  {
    value=0x01;
  }
  else{
    value=0x00;
  }
  pi_camera_reg_set(camera, HIMAX_QVGA_WIN_EN, &value);

  reset_exposure_calibration();
}

void himax_set_target_value(struct pi_device *camera, uint8_t value)
{
  // TODO (Jerome): check if we should leave it here
  // Cannot set the target value if we also reset AE (why?)
  // uint8_t s_value = 0x01;
  // pi_camera_reg_set(camera, HIMAX_AE_CTRL, &s_value);
  pi_camera_reg_set(camera, HIMAX_AE_TARGET_MEAN, &value);
  reset_exposure_calibration();
}

void himax_configure(struct pi_device *camera, unsigned int exposure_calibration_size)
{
  // printf("Configuring the camera\n");
  uint8_t set_value = 3;
  // uint8_t reg_value;
  pi_camera_reg_set(camera, IMG_ORIENTATION, &set_value);
  // pi_camera_reg_get(camera, IMG_ORIENTATION, &reg_value);
  // printf("img orientation %d\n",reg_value);

  // set_value=0x03;
  // pi_camera_reg_set(camera, HIMAX_MAX_AGAIN_BIN2, &set_value);
  // pi_camera_reg_set(camera, HIMAX_MAX_AGAIN_FULL, &set_value);

  set_value=0x00;
  pi_camera_reg_set(camera, HIMAX_FS_CTRL, &set_value);

  init_exposure_calibration(exposure_calibration_size);

}

#define MAX_CALIBRATION_SIZE 100
#define MIN_CALIBRATION_SIZE 3

static uint8_t  *a_gain;
static uint16_t *integration;
static uint16_t *d_gain;
static int n = -1; // not active
static int size;

static uint16_t median16(int n, uint16_t x[]) {
    uint16_t temp;
    int i, j;
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }
    return x[n/2];
}

static uint8_t median8(int n, uint8_t x[]) {
    uint8_t temp;
    int i, j;
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }
    return x[n/2];
}

static void reset_exposure_calibration()
{
  n = 0;
}

void init_exposure_calibration(unsigned int calibration_size)
{
  // printf("size calibration_size %d %d\n", size , calibration_size);
  if(size == calibration_size) {
    reset_exposure_calibration();
    return;
  }
  if(size){
    if(a_gain) pi_fc_l1_free(a_gain, size);
    if(d_gain) pi_fc_l1_free(d_gain, size * 2);
    if(integration) pi_fc_l1_free(integration, size * 2);
  }
  reset_exposure_calibration();
  if(calibration_size <= 0)
  {
    size = 0;
    return;
  }
  if(calibration_size < MIN_CALIBRATION_SIZE){
    printf("Requested calibration size %s is too small; will set it to %d\n",
        calibration_size, MIN_CALIBRATION_SIZE);
    size = MIN_CALIBRATION_SIZE;
  }
  else if(calibration_size > MAX_CALIBRATION_SIZE)
  {
    printf("Requested calibration size %s is too large; will cap it to %d\n",
        calibration_size, MAX_CALIBRATION_SIZE);
    size = MAX_CALIBRATION_SIZE;
  }
  else{
    size = calibration_size;
  }

  // create three arrays for storing analog gain, digital gain, and integration time. The size is 8-16bit * NUM_CALIBRATION_FRAMES
  a_gain = (uint8_t *)pi_fc_l1_malloc(size);
  d_gain = (uint16_t *)pi_fc_l1_malloc(size * 2);
  integration = (uint16_t *)pi_fc_l1_malloc(size * 2);
  if(!a_gain || !d_gain || !integration){
    printf("Could not allocate exposure buffer of size %d", size);
    if(a_gain) pi_fc_l1_free(a_gain, size);
    if(d_gain) pi_fc_l1_free(d_gain, size * 2);
    if(integration) pi_fc_l1_free(integration, size * 2);
    size = 0;
  }
  // printf("size calibration_size %d %d %d\n", (int) a_gain , (int) d_gain, (int) integration);

}

#define DEBUG_EXPOSURE_CALIBRATION 1 // this prints the reg values !
void himax_update_exposure(struct pi_device *camera)
{

  // printf("size %d n %d\n", size, n);
  if(size <= 0) return;
  if(n>=size)
  {
#ifdef DEBUG_EXPOSURE_CALIBRATION
    get_himax_exp_params(&camera);
#endif
    return;
  }

  // Force the ae control to be on
  uint8_t value8;
  if(n < 3)
  {
    _himax_enable_ae(camera, 1);
    // printf("_himax_enable_ae -------------\n");
  }

  pi_camera_reg_get16(camera, HIMAX_INTEGRATION_L, HIMAX_INTEGRATION_H, integration+n);
  pi_camera_reg_get(camera, HIMAX_ANAprintf_GAIN, a_gain+n);
  pi_camera_reg_get16(camera, HIMAX_DIGITAL_GAIN_L, HIMAX_DIGITAL_GAIN_H, d_gain+n);

  n++;

  if(n==size)
  {
    //calculate average values over NUM_CALIBRATION_FRAMES
    uint16_t m_integration = median16(size, integration);
    uint16_t m_dgain = median16(size, d_gain);
    uint8_t  m_again = median8(size, a_gain);

#ifdef DEBUG_EXPOSURE_CALIBRATION
    printf("---- AE Cumulated values ----\n");
    printf("integration time:");
    for (size_t i = 0; i < size; i++) {
      printf("%d, ", integration[i]);
    }
    printf("=> median %d \n", m_integration);
    printf("anaprintf gain: ");
    for (size_t i = 0; i < size; i++) {
      printf("%d, ", a_gain[i]);
    }
    printf("=> median %d\n", m_again);
    printf("digital gain: ");
    for (size_t i = 0; i < size; i++) {
      printf("%d, ", d_gain[i]);
    }
    printf("=> median %d\n", m_dgain);
    printf("-----------------------------\n");
#endif
    printf("Disable AE control after %d steps and fix registers:\n"
        "\t- INTEGRATION:     0x%04x\n"
        "\t- DIGITAL GAIN:    0x%04x\n"
        "\t- ANAprintf GAIN:  0x%02x\n", size, m_integration, m_dgain, m_again);

    pi_camera_reg_set(camera, HIMAX_GRP_PARAM_HOLD, 0x1); // commit
    pi_time_wait_us(100000);
    _himax_enable_ae(camera, 0);
    pi_camera_reg_set16(camera, HIMAX_INTEGRATION_L, HIMAX_INTEGRATION_H, &m_integration);
    pi_camera_reg_set16(camera, HIMAX_DIGITAL_GAIN_L, HIMAX_DIGITAL_GAIN_H, &m_dgain);
    pi_camera_reg_set(camera, HIMAX_ANAprintf_GAIN, &m_again);
    pi_camera_reg_set(camera, HIMAX_GRP_PARAM_HOLD, 0x0); // endcommit
    pi_time_wait_us(100000);
  }
}


void get_himax_exp_params(struct pi_device *camera){
    uint8_t value8;
    uint16_t value16;
    printf("Himax params:\n");
    pi_camera_reg_get16(camera, HIMAX_INTEGRATION_L, HIMAX_INTEGRATION_H, &value16);
    printf("\tintegration time: 0x%04x\n", value16);
    pi_camera_reg_get16(camera, HIMAX_DIGITAL_GAIN_L, HIMAX_DIGITAL_GAIN_H, &value16);
    printf("\tdigital gain:     0x%04x\n", value16);
    pi_camera_reg_get(camera, HIMAX_ANAprintf_GAIN, &value8);
    printf("\tanalog gain:   0x%02x\n", value8);
}


void write_himax_exp_params(struct pi_device *camera, uint16_t integration_value16, uint16_t d_gain_value16, uint8_t a_gain_value){
    /**
     * digital gain
     *    1x -> DIGITAL_GAIN_H    0x01
     *          DIGITAL_GAIN__L   0x00
     * analog gain
     *    1x -> 0x00
     *    2x -> 0x10
     *    4x -> 0x20
     *    8x -> 0x30
    */

    printf("Setting:\n");
    printf("\tintegration time: 0x%04x\n", integration_value16);
    printf("\tdigital gain:     0x%04x\n", d_gain_value16);
    printf("\tanaprintf gain:   0x%02x\n", a_gain_value);
    // start commit
    pi_camera_reg_set(camera, HIMAX_GRP_PARAM_HOLD, 0x1);
    pi_time_wait_us(100000);

    // disable Auto-Exposure
    _himax_enable_ae(camera, 0);
    // set registers
    pi_camera_reg_set16(camera, HIMAX_INTEGRATION_L, HIMAX_INTEGRATION_H, &integration_value16);
    pi_camera_reg_set16(camera, HIMAX_DIGITAL_GAIN_L, HIMAX_DIGITAL_GAIN_H, &d_gain_value16);
    pi_camera_reg_set(camera,   HIMAX_ANAprintf_GAIN, &a_gain_value);
    // end commit
    pi_camera_reg_set(camera, HIMAX_GRP_PARAM_HOLD, 0x0);
    pi_time_wait_us(100000);

    printf("Reding written registers:\n");
    get_himax_exp_params(camera);

}
