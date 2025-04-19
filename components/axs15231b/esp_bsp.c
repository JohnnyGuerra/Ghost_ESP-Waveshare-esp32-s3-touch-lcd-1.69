/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_rom_gpio.h"

#include "bsp_err_check.h"
#include "lv_port.h"
#include "display.h"
#include "esp_bsp.h"

static const char *TAG = "esp_bsp";

/* Waveshare 1.69" LCD Touch ESP32-S3R8 Configuration */
#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
#define LCD_H_RES               240
#define LCD_V_RES               280
#define LCD_BACKLIGHT_GPIO      15
#define LCD_RESET_GPIO          8
#define LCD_DC_GPIO             4
#define LCD_CS_GPIO             5
#define LCD_SCLK_GPIO           6
#define LCD_MOSI_GPIO           7
#define TOUCH_SCL_GPIO          10
#define TOUCH_SDA_GPIO          11
#define TOUCH_RST_GPIO          13
#define TOUCH_INT_GPIO          14
#endif

static lv_disp_t *disp;
static lv_indev_t *disp_indev = NULL;
static esp_lcd_touch_handle_t tp = NULL;   // LCD touch handle
static esp_lcd_panel_handle_t panel_handle = NULL;

static bool i2c_initialized = false;

/* I2C Initialization */
esp_err_t bsp_i2c_init(void)
{
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_SDA_GPIO,
        .scl_io_num = TOUCH_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // 400kHz
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_param_config(I2C_NUM_0, &i2c_conf));
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0));

    i2c_initialized = true;
    return ESP_OK;
}

/* SPI Initialization for LCD */
static esp_err_t bsp_lcd_spi_init(void)
{
    const spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_MOSI_GPIO,
        .miso_io_num = -1,
        .sclk_io_num = LCD_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2 + 8,
    };
    BSP_ERROR_CHECK_RETURN_ERR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    return ESP_OK;
}

/* Backlight Initialization */
static esp_err_t bsp_lcd_backlight_init(void)
{
    gpio_reset_pin(LCD_BACKLIGHT_GPIO);
    gpio_set_direction(LCD_BACKLIGHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_BACKLIGHT_GPIO, 1); // Turn on backlight
    return ESP_OK;
}

/* LCD Initialization */
static esp_err_t bsp_lcd_init(esp_lcd_panel_handle_t *panel)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_lcd_spi_init());

    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = LCD_CS_GPIO,
        .dc_gpio_num = LCD_DC_GPIO,
        .pclk_hz = 40 * 1000 * 1000, // 40MHz
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &io_handle));

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RESET_GPIO,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_st7789(io_handle, &panel_config, panel));

    esp_lcd_panel_reset(*panel);
    esp_lcd_panel_init(*panel);
    esp_lcd_panel_disp_on_off(*panel, true);

    return ESP_OK;
}

/* Touch Controller Initialization */
static esp_err_t bsp_touch_init(esp_lcd_touch_handle_t *touch)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    const esp_lcd_touch_config_t touch_config = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TOUCH_RST_GPIO,
        .int_gpio_num = TOUCH_INT_GPIO,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
    };
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_new_i2c(NULL, &touch_config, touch));
    return ESP_OK;
}

/* Display Initialization */
lv_disp_t *bsp_display_start(void)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_lcd_backlight_init());
    BSP_ERROR_CHECK_RETURN_NULL(bsp_lcd_init(&panel_handle));
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_init(&tp));

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_flush_cb;
    disp_drv.user_data = panel_handle;

    disp = lv_disp_drv_register(&disp_drv);

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_touch_cb;
    indev_drv.user_data = tp;

    disp_indev = lv_indev_drv_register(&indev_drv);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}
