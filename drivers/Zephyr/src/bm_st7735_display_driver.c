#define DT_DRV_COMPAT sitronix_bm_st7735

#include "bm_st7735_display_driver.h"
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(bm_st7735_display, CONFIG_BM_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/**
 * @brief  Set DCX pin
 * @note Pulls DCX pin low to send command and high to send data.
 */
static int bm_st7735r_set_dc_pin(const struct bm_st7735_config* config, int cmd_or_data)
{
    return gpio_pin_set_dt(&config->dc, cmd_or_data);
}

/**
 * @brief Transmit command.
 * @note Sends a command to the display.
 * @return error code.
 */
static int bm_st7735_transmit_cmd(const struct device* dev, uint8_t cmd)
{
    const struct bm_st7735_config* config = (struct bm_st7735_config*)dev->config;

    struct spi_buf           tx_buf  = { .buf = &cmd, .len = 1 };
    const struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

    int res = bm_st7735r_set_dc_pin(config, BM_ST7735_IS_COMMAND);
    res += spi_write_dt(&config->spi, &tx_bufs);

    return res;
}

/**
 * @brief Transmit data.
 * @note Used after a command is sent.
 * @note Sends data to the display, for setting parameters and configurations.
 * @return error code.
 */
static int bm_st7735_transmit_data(const struct device* dev,
    const uint8_t* tx_data, size_t tx_count)
{
    const struct bm_st7735_config* config = (struct bm_st7735_config*)dev->config;

    struct spi_buf           tx_buf  = { .buf = (void*)tx_data, .len = tx_count };
    const struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

    int res = bm_st7735r_set_dc_pin(config, BM_ST7735_IS_DATA);
    res += spi_write_dt(&config->spi, &tx_bufs);

    return res;
}

/**
 * @brief Reset display.
 * @note Reset the display by setting GPIO Reset Pin low.
 * @note If the GPIO pin is not configured, display is reset through SW CMD.
 * @return error code.
 */
static int bm_st7735_reset_display(const struct device* dev)
{
    LOG_DBG("Resetting display.");

    const struct bm_st7735_config* config = (struct bm_st7735_config*)dev->config;

    int res = 0;

    if (config->reset.port != NULL) {
        res += gpio_pin_set(config->reset.port, config->reset.pin, HIGH); // Set High
        res += gpio_pin_set(config->reset.port, config->reset.pin, LOW); // Set Low
        k_sleep(BM_ST7735_RESET_DURATION);
        res += gpio_pin_set(config->reset.port, config->reset.pin, HIGH); // Set High
    } else {
        res += bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_SWRESET);
    }

    if (res != 0)
        return res;

    k_sleep(BM_ST7735_RESET_DURATION);
    return 0;
}

/**
 * @brief Enter sleep mode.
 * @note Enter Minimum Power Consumption Mode.
 * @note DC converter, internal oscillator and panel driver circuit are stopped. Only the MCU interface and memory works with VDDI power supply. Contents of the memory are safe.
 * @return error code.
 */
static int bm_st7735_sleep_in(const struct device* dev)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_SLPIN);
    k_sleep(BM_ST7735_EXIT_SLEEP_DURATION);
    return res;
}

/**
 * @brief Exit sleep mode.
 * @return error code.
 */
static int bm_st7735_sleep_out(const struct device* dev)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_SLPOUT);
    k_sleep(BM_ST7735_EXIT_SLEEP_DURATION);
    return res;
}

/**
 * @brief Turn on Partial Display Mode
 * @note Exiting Normal Display Mode.
 * @return error code.
 */
static int bm_st7735_partial_mode_on(const struct device* dev)
{
    return bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_PTLON);
}

/**
 * @brief Turn on Normal Display Mode
 * @note Exiting Partial Display Mode.
 * @return error code.
 */
static int bm_st7735_normal_mode_on(const struct device* dev)
{
    return bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_NORON);
}

/**
 * @brief Turn Display Inversion Off.
 * @return error code.
 */
static int bm_st7735_inversion_off(const struct device* dev)
{
    return bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_INVOFF);
}

/**
 * @brief Turn Display Inversion On.
 * @note Inverts the colors of the display.
 * @note Refers to inverting the polarity of pixel voltages. Black becomes white, vice versa.
 * @return error code.
 */
static int bm_st7735_inversion_on(const struct device* dev)
{
    return bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_INVON);
}

/**
 * @brief Set the desired Gamma Curve.
 * @param gamma_curve 8 bits [-, -, -, -, GC3 GC2 GC1 GC0]
 * @return error code.
 */
static int bm_st7735_set_gamma_curve(const struct device* dev, uint8_t* gamma_curve)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_GAMSET);
    res += bm_st7735_transmit_data(dev, gamma_curve, 1);
    return res;
}

/**
 * @brief Turn display on.
 * @note The display is able to show maximum 262,144 colors.
 * @return error code.
 */
static int bm_st7735_display_on(const struct device* dev)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_DISPON);
    k_sleep(BM_ST7735_EXIT_SLEEP_DURATION);
    return res;
}

/**
 * @brief Turn display off.
 * @note Both VDD and VDDI are removed. Backlight is still on.
 * @return error code3.
 */
static int bm_st7735_display_off(const struct device* dev)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_DISPOFF);
    k_sleep(BM_ST7735_EXIT_SLEEP_DURATION);
    return res;
}

/**
 * @brief Set the column intervall of the RAM.
 * @param x_start Start of RAM column. Often 0.
 * @param x_end End of RAM column. Often display width.
 * @note Designed for the specific display.
 * @note Need to assert 0 < XS < XE < max_col. Max_col is defined by MV and GM.
 * @return error code.
 */
static int bm_st7735_set_column_interval(const struct device* dev, const uint16_t x_start, const uint16_t x_end)
{
    __ASSERT(x_start >= 0x0000, "Column start is smaller than zero.");
    __ASSERT(x_end > x_start, "End Column is smaller than start column.");

    uint8_t tx_data[4];
    tx_data[3] = (uint8_t)x_end;
    tx_data[2] = (uint8_t)(x_end >> 8);
    tx_data[1] = (uint8_t)x_start;
    tx_data[0] = (uint8_t)(x_start >> 8);

    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_CASET);
    res += bm_st7735_transmit_data(dev, tx_data, sizeof(tx_data));
    return res;
}

/**
 * @brief Set the row intervall of the RAM
 * @param y_start Start of RAM row. Often 0.
 * @param y_end End of RAM row. Often display height.
 * @note Designed for the specific display.
 * @return error code.
 */
static int bm_st7735_set_row_interval(const struct device* dev, const uint16_t y_start, const uint16_t y_end)
{
    __ASSERT(y_start >= 0x0000, "Start row is smaller than zero.");
    __ASSERT(y_end > y_start, "End row is smaller than start row.");

    uint8_t tx_data[4];
    tx_data[3] = (uint8_t)y_end;
    tx_data[2] = (uint8_t)(y_end >> 8);
    tx_data[1] = (uint8_t)y_start;
    tx_data[0] = (uint8_t)(y_start >> 8);

    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_RASET);
    res += bm_st7735_transmit_data(dev, tx_data, sizeof(tx_data));
    return res;
}

/**
 * @brief Write image to the ram of the display.
 * @param x  ?
 * @param y ?
 * @param desc ?
 * @param buf Buf comprise a 128x128x24 bit image filling the display.
 * @note Default: Contents of memory is set randomly at power on sequence.
 * TODO: Be able to set start row and columns address to send from.
 * @return error code.
 */
static int bm_st7735_write_ram(const struct device* dev,
    const uint16_t                                  x,
    const uint16_t                                  y,
    const struct display_buffer_descriptor*         desc,
    const void*                                     buf)
{
    const struct bm_st7735_config* config   = (struct bm_st7735_config*)dev->config;
    const uint8_t*                 data_buf = (uint8_t*)(buf);
    int                            res      = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_RAMWR);

    unsigned num_bytes = (config->width * config->height * BM_ST7735_NUM_BITS_PER_PIXEL) / BM_ST7735_NUM_BITS_PER_BYTE;
    res += bm_st7735_transmit_data(dev, data_buf, num_bytes);
    return res;
}

/**
 * @brief Test function for printing same color over the whole display.
 * @return error code.
 */
static int bm_st7735_write_ram_color_test(const struct device* dev,
    const uint16_t                                             x,
    const uint16_t                                             y,
    const struct display_buffer_descriptor*                    desc,
    const void*                                                buf)
{
    const struct bm_st7735_config* config   = (struct bm_st7735_config*)dev->config;
    const uint8_t*                 data_buf = (uint8_t*)(buf);
    int                            res      = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_RAMWR);

    unsigned num_pixels = config->width * config->height;
    for (unsigned i = 0; i < num_pixels; i++) {
        res += bm_st7735_transmit_data(dev, data_buf, 3);
    }
    return res;
}

/**
 * @brief  Set color settings for 4K, 65K, 262K.
 * @param color_settings ?
 * @note Defines LUT color depth conversion between 12-16bit and 16-18bit.
 * @return error code.
 */
static int bm_st7735_set_color_settings(const struct device* dev, const uint8_t* color_settings)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_RGBSET);
    res += bm_st7735_transmit_data(dev, color_settings, 127);

    return res;
}

/**
 * @brief  Set the memory access control settings. Sets how to interact with the memory.
 * @param mem_access_config 8 bit: [MY, MX, MV, ML, RGB, MH, -, -]
 * @note MY: Row Address Order: 0=top2bottom row, 1=bottom2top row.
 * @note MX: Column Address Order.0=left2right col, 1=right2left col.
 * @note MV: Row/Column Address order: 0:noExchange(normal order) (X,Y), 1:exchanged(Y,X). Row becomes col and vice versa. w becomes height, and vice versa. 132x162 -> 162x132.
 * @note ML: Vertical Refresh Order: 0=top2bottom, 1=bottom2top.
 * @note RGB: RGB-BGR Order: 0=BGR, 1=RGB.
 * @note MH: Horizontal Refresh Order: 0=left2right, 1:right2left.
 * @return error code.
 */
static int bm_st7735_set_mem_access_ctr(const struct device* dev, const uint8_t* mem_access_config)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_MADCTL);
    res += bm_st7735_transmit_data(dev, mem_access_config, 1);
    return res;
}

/**
 * @brief  Exit IDLE mode.
 * @note LCD display 4k, 65k, 262k colors, and normal frame frequency is applied.
 * @return error code.
 */
static int bm_st7735_idle_mode_off(const struct device* dev)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_IDMOFF);
    return res;
}

/**
 * @brief  Enter IDLE mode.
 * @note Color expression reduced, 8bit color depth.
 * @return error code.
 */
static int bm_st7735_idle_mode_on(const struct device* dev)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_IDMON);
    return res;
}

/**
 * @brief  Get the displays capabilities:
 * @note Resolution (x,y)
 * @note Pixel format (bm_st7735_display_pixel_format)
 * @return error code.
 */
static void bm_st7735_get_capabilities(const struct device* dev, struct bm_st7735_capabilities* capabilities)
{
    const struct bm_st7735_config* config = dev->config;

    memset(capabilities, 0, sizeof(struct display_capabilities));
    capabilities->x_resolution = config->width;
    capabilities->y_resolution = config->height;

    capabilities->current_pixel_format = config->colmod;
}

/**
 * @brief  Set pixel format/color mode.
 * @note Three different settings: 12bit, 16bit, 18bit data transfer.
 * @note  Defined in enum: bm_st7735_display_pixel_format.
 * @return error code.
 */
static int bm_st7735_set_pixel_format(const struct device* dev, const enum bm_st7735_display_pixel_format pixel_format)
{
    __ASSERT((pixel_format == PIXEL_FORMAT_666) | (pixel_format == PIXEL_FORMAT_565) | (pixel_format == PIXEL_FORMAT_444), "Pixel format is not supported.");

    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_COLMOD);
    res += bm_st7735_transmit_data(dev, &pixel_format, 1);
    return res;
}

/**
 * @brief  Set gamma correction characteristics settings.
 * @note Set 16 registers for both positive and negative polarity.
 * @return error code.
 */
static int bm_st7735_set_gamma_correction(const struct device* dev, const uint8_t* pos_gm_correction_data, const uint8_t* neg_gm_correction_data)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_GMCTRP1);
    res += bm_st7735_transmit_data(dev, pos_gm_correction_data, 16);

    res += bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_GMCTRN1);
    res += bm_st7735_transmit_data(dev, neg_gm_correction_data, 16);
    return res;
}

/**
 * @brief  Set VCOM control settings.
 * @param vcom_config ? Set one register.
 * @return error code.
 */
static int bm_st7735_set_vcom_ctr(const struct device* dev, const uint8_t vcom_config)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_VMCTR1);
    res += bm_st7735_transmit_data(dev, &vcom_config, 1);
    return res;
}

/**
 * @brief  Set power control settings.
 * @note Set five registers.
 * @return error code.
 */
static int bm_st7735_set_pwr_ctr(const struct device* dev, const uint8_t* pwr1, const uint8_t* pwr2, const uint8_t* pwr3, const uint8_t* pwr4, const uint8_t* pwr5)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_PWCTR1);
    res += bm_st7735_transmit_data(dev, pwr1, 3);

    res += bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_PWCTR2);
    res += bm_st7735_transmit_data(dev, pwr2, 1);

    res += bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_PWCTR3);
    res += bm_st7735_transmit_data(dev, pwr3, 2);

    res += bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_PWCTR4);
    res += bm_st7735_transmit_data(dev, pwr4, 2);

    res += bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_PWCTR5);
    res += bm_st7735_transmit_data(dev, pwr5, 2);
    return res;
}

/**
 * @brief  Set frame rate control settings.
 * @note Set three different registers.
 * @param frm1 Frame frequency of full colors under normal mode operation.
 * @param frm2 Set frame frequency in IDLE Mode.
 * @param frm3 Set frame frequency in partial mode/full colors.
 * @return error code.
 */
static int bm_st7735_set_frame_rate_ctr(const struct device* dev, const uint8_t* frm1, const uint8_t* frm2, const uint8_t* frm3)
{
    int res = bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_FRMCTR1);
    res += bm_st7735_transmit_data(dev, frm1, 3);

    res += bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_FRMCTR2);
    res += bm_st7735_transmit_data(dev, frm2, 1);

    res += bm_st7735_transmit_cmd(dev, BM_ST7735_CMD_FRMCTR3);
    res += bm_st7735_transmit_data(dev, frm3, 2);

    return res;
}

/**
 * @brief  Set GPIO reset and dc pin to output.
 * @return error code.
 */
static int bm_st7735_set_spi_wiring(const struct device* dev)
{
    const struct bm_st7735_config* config = (struct bm_st7735_config*)dev->config;

    int res = gpio_pin_configure(config->reset.port, config->reset.pin, GPIO_OUTPUT);
    res += gpio_pin_configure(config->dc.port, config->dc.pin, GPIO_OUTPUT);
    return res;
}

/**
 * @brief  Sets display params given in device configuration in overlay.
 * TODO: Use caset and raset when defining memory area.
 * @return error code.
 */
static int bm_st7735_set_lcd_params(const struct device* dev)
{
    const struct bm_st7735_config* config = (struct bm_st7735_config*)dev->config;
    int                            res    = 0;

    res += bm_st7735_set_mem_access_ctr(dev, &(config->madctl));

    res += bm_st7735_set_column_interval(dev, 0, config->width);

    res += bm_st7735_set_row_interval(dev, 0, config->height);

    res += bm_st7735_set_pixel_format(dev, config->colmod);

    res += bm_st7735_set_gamma_correction(dev, config->gmctrp1, config->gmctrp1);

    res += bm_st7735_set_vcom_ctr(dev, config->vmctr1);

    res += bm_st7735_set_pwr_ctr(dev, config->pwctr1, config->pwctr2, config->pwctr3, config->pwctr4, config->pwctr5);

    res += bm_st7735_set_frame_rate_ctr(dev, config->frmctr1, config->frmctr2, config->frmctr3);

    return res;
}

/**
 * @brief  Initialise the display.
 * @note 1. Check SPI, 2. Set wiring, 3. Reset display,
 * @note 4. Turn on display, 5. Exit sleep, 6. Set LCD parameters.
 * @return error code.
 */
static int bm_st7735_display_init(const struct device* dev)
{
    const struct bm_st7735_config* config = (struct bm_st7735_config*)dev->config;

    if (!device_is_ready(config->spi.bus)) {
        LOG_ERR("SPI bus not ready");
        return ENODEV;
    }

    if (bm_st7735_set_spi_wiring(dev) != 0)
        return EIO;

    if (bm_st7735_reset_display(dev) != 0)
        return EIO;

    if (bm_st7735_display_on(dev) != 0)
        return EIO;

    if (bm_st7735_sleep_out(dev) != 0)
        return EIO;

    if (bm_st7735_set_lcd_params(dev) != 0)
        return EIO;

    LOG_INF("%s ready", dev->name);
    return 0;
}

#ifdef CONFIG_PM_DEVICE
/**
 * @brief  Power management function.
 * @note Controls display on/off and sleep in/out.
 * @return error code.
 */
static int bm_st7735_pm_action(const struct device* dev,
    enum pm_device_action                           action)
{
    int res = 0;

    switch (action) {
    case PM_DEVICE_ACTION_RESUME:
        /* Activate the display*/
        res += bm_st7735_sleep_out(dev);
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        /* Suspend the display. */
        res += bm_st7735_sleep_in(dev);
        break;
    case PM_DEVICE_ACTION_TURN_OFF:
        /* Turn off the display. */
        res += bm_st7735_display_off(dev);
        break;
    case PM_DEVICE_ACTION_TURN_ON:
        /* Turn on the display. */
        res += bm_st7735_display_on(dev);
        break;
    default:
        return -ENOTSUP;
    }
    return res;
}
#endif /* CONFIG_PM_DEVICE */

static struct bm_st7735_config bm_st7735_config = {
    .spi             = SPI_DT_SPEC_INST_GET(DISPLAY_INSTANCE_NUMBER, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0), // Should we use bus instead of SPI.
    .reset           = GPIO_DT_SPEC_GET(DT_NODELABEL(bm_st7735), reset_gpios),
    .dc              = GPIO_DT_SPEC_GET(DT_NODELABEL(bm_st7735), cmd_data_gpios),
    .height          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, height),
    .width           = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, width),
    .madctl          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, madctl),
    .colmod          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, colmod),
    .caset           = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, caset),
    .raset           = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, raset),
    .vmctr1          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, vmctr1),
    .invctr          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, invctr),
    .pwctr1          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, pwctr1),
    .pwctr2          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, pwctr2),
    .pwctr3          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, pwctr3),
    .pwctr4          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, pwctr4),
    .pwctr5          = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, pwctr5),
    .frmctr1         = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, frmctr1),
    .frmctr2         = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, frmctr2),
    .frmctr3         = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, frmctr3),
    .gmctrp1         = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, gmctrp1),
    .gmctrn1         = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, gmctrn1),
    .inversion_on    = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, inversion_on),
    .rgb_is_inverted = DT_INST_PROP(DISPLAY_INSTANCE_NUMBER, rgb_is_inverted),
};

static const struct bm_st7735_driver_api bm_st7735_api = {
    .get_capabilities     = bm_st7735_get_capabilities,
    .write_image          = bm_st7735_write_ram,
    .write_image_test     = bm_st7735_write_ram_color_test,
    .inversion_on         = bm_st7735_inversion_on,
    .inversion_off        = bm_st7735_inversion_off,
    .set_color_conversion = bm_st7735_set_color_settings,
    .set_pixel_format     = bm_st7735_set_pixel_format,
    .set_gamma_curve      = bm_st7735_set_gamma_curve,
    .normal_mode_on       = bm_st7735_normal_mode_on,
    .partial_mode_on      = bm_st7735_partial_mode_on,
    .idle_mode_on         = bm_st7735_idle_mode_on,
    .idle_mode_off        = bm_st7735_idle_mode_off,
};

PM_DEVICE_DT_DEFINE(bm_st7735, bm_st7735_pm_action);

DEVICE_DT_INST_DEFINE(DISPLAY_INSTANCE_NUMBER,
    &bm_st7735_display_init,
    NULL,
    PM_DEVICE_DT_GET(bm_st7735),
    &bm_st7735_config,
    APPLICATION,
    CONFIG_APPLICATION_INIT_PRIORITY,
    &bm_st7735_api);

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)