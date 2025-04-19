/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: Waveshare 1.69" LCD Touch ESP32-S3R8
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "lvgl.h"
#include "lv_port.h"

#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH

/**************************************************************************************************
 *  Pinout for Waveshare 1.69" LCD Touch ESP32-S3R8
 **************************************************************************************************/
#define BSP_I2C_NUM                     (I2C_NUM_0)
#define BSP_I2C_CLK_SPEED_HZ            400000

#define BSP_LCD_SPI_HOST                (SPI2_HOST)

#define BSP_LCD_H_RES                   (240)  // Horizontal resolution
#define BSP_LCD_V_RES                   (280)  // Vertical resolution

#define BSP_LCD_BACKLIGHT_GPIO          (15)   // Backlight GPIO pin
#define BSP_LCD_RESET_GPIO              (8)    // Reset GPIO pin
#define BSP_LCD_DC_GPIO                 (4)    // Data/Command GPIO pin
#define BSP_LCD_CS_GPIO                 (5)    // Chip Select GPIO pin
#define BSP_LCD_SCLK_GPIO               (6)    // SPI Clock GPIO pin
#define BSP_LCD_MOSI_GPIO               (7)    // SPI MOSI GPIO pin

#define BSP_TOUCH_SCL_GPIO              (10)   // Touch I2C Clock GPIO pin
#define BSP_TOUCH_SDA_GPIO              (11)   // Touch I2C Data GPIO pin
#define BSP_TOUCH_RST_GPIO              (13)   // Touch Reset GPIO pin
#define BSP_TOUCH_INT_GPIO              (14)   // Touch Interrupt GPIO pin

#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< Configuration for the LVGL port */
    uint32_t buffer_size;           /*!< Size of the buffer for the screen in pixels */
    lv_disp_rot_t rotate;           /*!< Rotation configuration for the display */
} bsp_display_cfg_t;

/**
 * @brief Initialize I2C driver
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *      - ESP_FAIL              I2C driver installation error
 *
 */
esp_err_t bsp_i2c_init(void);

/**
 * @brief Deinitialize I2C driver and free its resources
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *
 */
esp_err_t bsp_i2c_deinit(void);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller, and starts the LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set().
 *
 * @param cfg Display configuration
 *
 * @return Pointer to LVGL display or NULL when an error occurred
 */
lv_disp_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

/**
 * @brief Get pointer to input device (touch, buttons, etc.)
 *
 * @note The LVGL input device is initialized in bsp_display_start() function.
 *
 * @return Pointer to LVGL input device or NULL when not initialized
 */
lv_indev_t *bsp_display_get_input_dev(void);

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void bsp_display_unlock(void);

/**
 * @brief Set display brightness
 *
 * Brightness is controlled with a PWM signal to the backlight GPIO pin.
 * Display must be initialized by calling bsp_display_start_with_config().
 *
 * @param brightness_percent Brightness in [%]
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_brightness_set(int brightness_percent);

/**
 * @brief Turn on display backlight
 *
 * Display must be initialized by calling bsp_display_start_with_config().
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_backlight_on(void);

/**
 * @brief Turn off display backlight
 *
 * Display must be initialized by calling bsp_display_start_with_config().
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   Parameter error
 */
esp_err_t bsp_display_backlight_off(void);

#ifdef __cplusplus
}
#endif
