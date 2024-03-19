#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/*
    Read functionality is not supported on this driver.
*/

// Instance number same as defined in overlay
#define DISPLAY_INSTANCE_NUMBER 0

// Helpers
#define HIGH 1
#define LOW  0

// Timing
#define BM_ST7735_RESET_DURATION      K_MSEC(1)
#define BM_ST7735_EXIT_SLEEP_DURATION K_MSEC(120)

// Messaging
#define BM_ST7735_IS_COMMAND         LOW
#define BM_ST7735_IS_DATA            HIGH
#define BM_ST7735_NUM_BITS_PER_PIXEL 24
#define BM_ST7735_NUM_BITS_PER_BYTE  8

/* System Function Commands*/
#define BM_ST7735_CMD_SWRESET 0x01 // Software Reset.

#define BM_ST7735_CMD_RDDID     0x04 // Read Display ID
#define BM_ST7735_CMD_RDDST     0x09 // Read Display Status.
#define BM_ST7735_CMD_RDDPM     0x0A // Read DIsplay Power Mode
#define BM_ST7735_CMD_RDDMADCTL 0x0B // Read Display MADCTL(Memory Access Control)
#define BM_ST7735_CMD_RDDCOLMOD 0x0C // Read Display Pixel Format (Color Mode)
#define BM_ST7735_CMD_RDDIM     0x0D // Read Display Image Mode
#define BM_ST7735_CMD_RDDSM     0x0E // Read Display Signal Mode

#define BM_ST7735_CMD_SLPIN  0x10 // Sleep in.
#define BM_ST7735_CMD_SLPOUT 0x11 // Sleep out.
#define BM_ST7735_CMD_PTLON  0x12 // Partial Display Mode On.
#define BM_ST7735_CMD_NORON  0x13 // Normal Display Mode On.
#define BM_ST7735_CMD_INVOFF 0x20 // Display Inversion Off.
#define BM_ST7735_CMD_INVON  0x21 // Display Inversion on.
#define BM_ST7735_CMD_GAMSET 0x26 // Gamma Set.

#define BM_ST7735_CMD_DISPOFF 0x28 // Enter display off mode.
#define BM_ST7735_CMD_DISPON  0x29 // Enter display on mode.
#define BM_ST7735_CMD_CASET   0x2A // Column Address Set.
#define BM_ST7735_CMD_RASET   0x2B // Row Address Set.

#define BM_ST7735_CMD_RAMWR  0x2C // Memory write
#define BM_ST7735_CMD_RGBSET 0x2D // Color setting for 4K, 65K, 262K.

#define BM_ST7735_CMD_RAMRD  0x2E // Memory read
#define BM_ST7735_CMD_PTLAR  0x30 // NOT IMPLEMENTED. Partial Area. Defines/sets partial mode's display area.
#define BM_ST7735_CMD_TEOFF  0x34 // NOT IMPLEMENTED. Tearing effect line OFF. Tearing effect is a feature in TFT displays.
#define BM_ST7735_CMD_TEON   0x35 // NOT IMPLEMENTED. Tearing effect line ON. Provides a signal indicating the start of a new frame or a specific point in the display refresh cycle.
#define BM_ST7735_CMD_MADCTL 0x36 // Memory Data Access Control.

#define BM_ST7735_CMD_IDMOFF 0x38 // Idle Mode OFF.
#define BM_ST7735_CMD_IDMON  0x39 // Idle Mode ON.
#define BM_ST7735_CMD_COLMOD 0x3A // Interface Pixel Format.
#define BM_ST7735_CMD_RDID1  0xDA // Read ID1 Value. Manufacturer ID.
#define BM_ST7735_CMD_RDID2  0xDB // Read ID2 Value. LCD module/driver version ID.
#define BM_ST7735_CMD_RDID3  0xDC // Read ID3 Value. LCD module/driver ID

/* Panel Function Commands. Only used in the initialisation of the display. */
#define BM_ST7735_CMD_FRMCTR1 0xB1 // Frame Rate Control.
#define BM_ST7735_CMD_FRMCTR2 0xB2 // Frame Rate Control.
#define BM_ST7735_CMD_FRMCTR3 0xB3 // Frame Rate Control.

#define BM_ST7735_CMD_INVCTR 0xB4 // NOT IMPLEMENTED. Display Inversion Control. Set Line or Dot inversion in each operating mode.

#define BM_ST7735_CMD_PWCTR1 0xC0 // Power Control 1.
#define BM_ST7735_CMD_PWCTR2 0xC1 // Power Control 2.
#define BM_ST7735_CMD_PWCTR3 0xC2 // Power Control 3.
#define BM_ST7735_CMD_PWCTR4 0xC3 // Power Control 4.
#define BM_ST7735_CMD_PWCTR5 0xC4 // Power Control 5.

#define BM_ST7735_CMD_VMCTR1  0xC5 // VCOM Control 1.
#define BM_ST7735_CMD_VMOFCTR 0xC7 // NOT IMPLEMENTED. VCOM Offset Control. Set VCOM Voltage level for reduce flicker issue.

#define BM_ST7735_CMD_WRID2 0xD1 // NOT IMPLEMENTED. Write ID2 Value. Module version.
#define BM_ST7735_CMD_WRID3 0xD2 // NOT IMPLEMENTED. Write ID3 Value. Project code module.

#define BM_ST7735_CMD_NVFCTR1 0xD9 // NOT IMPLEMENTED. NVM Control Status. Command controlling status of NVM by setting params.
#define BM_ST7735_CMD_NVFCTR2 0xDE // NOT IMPLEMENTED. NVM Read Command
#define BM_ST7735_CMD_NVFCTR3 0xDF // NOT IMPLEMENTED. NVM Write Command

#define BM_ST7735_CMD_GMCTRP1 0xE0 // Gamma (+polarity) Correction Characteristics Setting
#define BM_ST7735_CMD_GMCTRN1 0xE1 // Gamma (-polarity) Correction Characteristics Setting

struct bm_st7735_config {
    struct spi_dt_spec  spi;
    struct gpio_dt_spec reset;
    struct gpio_dt_spec dc;
    uint16_t            height;
    uint16_t            width;
    uint8_t             madctl;
    uint8_t             colmod;
    uint8_t             caset[4];
    uint8_t             raset[4];
    uint8_t             vmctr1;
    uint8_t             invctr;
    uint8_t             pwctr1[3];
    uint8_t             pwctr2[1];
    uint8_t             pwctr3[2];
    uint8_t             pwctr4[2];
    uint8_t             pwctr5[2];
    uint8_t             frmctr1[3];
    uint8_t             frmctr2[3];
    uint8_t             frmctr3[6];
    uint8_t             gmctrp1[16];
    uint8_t             gmctrn1[16];
    bool                inversion_on;
    bool                rgb_is_inverted;
};

enum bm_st7735_display_pixel_format {
    PIXEL_FORMAT_666 = 0x06, // 18 bit color mode
    PIXEL_FORMAT_565 = 0x05, // 16 bit color mode
    PIXEL_FORMAT_444 = 0x03, // 12 bit color mode
};

struct bm_st7735_capabilities {
    /** Display resolution in the X direction */
    uint16_t x_resolution;
    /** Display resolution in the Y direction */
    uint16_t y_resolution;
    /** Currently active pixel format for the display */
    enum bm_st7735_display_pixel_format current_pixel_format;
};

/**
 * @typedef bm_st7735_inversion_off_api
 * @brief Callback API to display inversion off
 * See bm_st7735_inversion_off() for argument description
 */
typedef int (*bm_st7735_inversion_off_api)(const struct device* dev);

/**
 * @typedef bm_st7735_inversion_on_api
 * @brief Callback API to set display inversion on.
 * See bm_st7735_inversion_on() for argument description
 */
typedef int (*bm_st7735_inversion_on_api)(const struct device* dev);

/**
 * @typedef bm_st7735_set_color_config_api
 * @brief Callback API to bm_st7735_set_color_config
 * See bm_st7735_set_color_config() for argument description
 */
typedef int (*bm_st7735_set_color_config_api)(const struct device* dev, const uint8_t* color_settings_buf);

/**
 * @typedef display_get_capabilities_api
 * @brief Callback API to get display capabilities
 * See bm_st7735_display_get_capabilities() for argument description
 */
typedef void (*bm_st7735_get_capabilities_api)(const struct device* dev, struct bm_st7735_capabilities* capabilities);

/**
 * @typedef bm_st7735_et_pixel_format_api
 * @brief Callback API to set pixel format or color mode.
 * See bm_st7735_set_pixel_format() for argument description
 */
typedef int (*bm_st7735_set_pixel_format_api)(const struct device* dev, const enum bm_st7735_display_pixel_format pixel_format);

/**
 * @typedef bm_st7735_set_gamma_curve_api
 * @brief Callback API to set gamma curve.
 *  See bm_st7735_set_gamma_curve() for argument description
 */
typedef int (*bm_st7735_set_gamma_curve_api)(const struct device* dev, uint8_t* gamma_curve);

/**
 * @typedef bm_st7735_idle_mode_on_api
 * @brief Callback API to turn IDLE Mode on.
 *  See bm_st7735_idle_mode_on_api() for argument description
 */
typedef int (*bm_st7735_idle_mode_on_api)(const struct device* dev);

/**
 * @typedef bm_st7735_idle_mode_off_api
 * @brief Callback API to turn IDLE Mode off.
 *  See bm_st7735_idle_mode_off_api() for argument description
 */
typedef int (*bm_st7735_idle_mode_off_api)(const struct device* dev);

/**
 * @typedef bm_st7735_partial_mode_on_api
 * @brief Callback API to turn Partial Mode on.
 *  See bm_st7735_partial_mode_on_api() for argument description
 */
typedef int (*bm_st7735_partial_mode_on_api)(const struct device* dev);

/**
 * @typedef bm_st7735_normal_mode_on_api
 * @brief Callback API to turn Normal Mode off.
 *  See bm_st7735_normal_mode_on_api() for argument description
 */
typedef int (*bm_st7735_normal_mode_on_api)(const struct device* dev);

/* st7735 display does not support read commands. */
struct bm_st7735_driver_api {
    bm_st7735_get_capabilities_api get_capabilities;
    display_write_api              write_image;
    display_write_api              write_image_test;
    bm_st7735_inversion_on_api     inversion_on;
    bm_st7735_inversion_off_api    inversion_off;
    bm_st7735_set_color_config_api set_color_conversion;
    bm_st7735_set_pixel_format_api set_pixel_format;
    bm_st7735_set_gamma_curve_api  set_gamma_curve;
    bm_st7735_normal_mode_on_api   normal_mode_on;
    bm_st7735_partial_mode_on_api  partial_mode_on;
    bm_st7735_idle_mode_on_api     idle_mode_on;
    bm_st7735_idle_mode_off_api    idle_mode_off;
};