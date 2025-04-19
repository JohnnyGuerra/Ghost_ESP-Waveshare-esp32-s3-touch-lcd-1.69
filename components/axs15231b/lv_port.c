/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"

#include "lv_port.h"
#include "lvgl.h"

#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
#include "esp_lcd_touch.h"
#endif

#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 4)) || (ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(5, 0, 0))
#define LVGL_PORT_HANDLE_FLUSH_READY 0
#else
#define LVGL_PORT_HANDLE_FLUSH_READY 1
#endif

static const char *TAG = "LVGL";

/*******************************************************************************
* Waveshare 1.69" LCD Touch ESP32-S3R8 Configuration
*******************************************************************************/
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

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct lvgl_port_ctx_s {
    SemaphoreHandle_t   lvgl_mux;
    esp_timer_handle_t  tick_timer;
    bool                running;
    int                 task_max_sleep_ms;
} lvgl_port_ctx_t;

typedef struct {
    esp_lcd_panel_io_handle_t io_handle;    /* LCD panel IO handle */
    esp_lcd_panel_handle_t    panel_handle; /* LCD panel handle */
    lv_disp_drv_t             disp_drv;     /* LVGL display driver */

    uint32_t                  trans_size;       /* Maximum size for one transport */
    lv_color_t                *trans_buf_1;     /* Buffer send to driver */
    lv_color_t                *trans_buf_2;     /* Buffer send to driver */
    lv_color_t                *trans_act;       /* Active buffer for sending to driver */
    SemaphoreHandle_t         trans_done_sem;   /* Semaphore for signaling idle transfer */
    lv_disp_rot_t             sw_rotate;        /* Panel software rotation mask */

    lvgl_port_wait_cb         draw_wait_cb;     /* Callback function for drawing */
} lvgl_port_display_ctx_t;

#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
typedef struct {
    esp_lcd_touch_handle_t  handle;        /* LCD touch IO handle */
    lv_indev_drv_t          indev_drv;     /* LVGL input device driver */
    lvgl_port_wait_cb       touch_wait_cb; /* Callback function for touch */
} lvgl_port_touch_ctx_t;
#endif

/*******************************************************************************
* Local variables
*******************************************************************************/
static lvgl_port_ctx_t lvgl_port_ctx;
static int lvgl_port_timer_period_ms = 5;

/*******************************************************************************
* Function definitions
*******************************************************************************/
static void lvgl_port_task(void *arg);
static esp_err_t lvgl_port_tick_init(void);
static void lvgl_port_task_deinit(void);

// LVGL callbacks
#if LVGL_PORT_HANDLE_FLUSH_READY
static bool lvgl_port_flush_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
#endif
static void lvgl_port_flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
static void lvgl_port_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
#endif

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t lvgl_port_init(const lvgl_port_cfg_t *cfg)
{
    esp_err_t ret = ESP_OK;
    ESP_GOTO_ON_FALSE(cfg, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(cfg->task_affinity < (configNUM_CORES), ESP_ERR_INVALID_ARG, err, TAG, "Bad core number for task! Maximum core number is %d", (configNUM_CORES - 1));

    memset(&lvgl_port_ctx, 0, sizeof(lvgl_port_ctx));

    /* LVGL init */
    lv_init();
    /* Tick init */
    lvgl_port_timer_period_ms = cfg->timer_period_ms;
    ESP_RETURN_ON_ERROR(lvgl_port_tick_init(), TAG, "");
    /* Create task */
    lvgl_port_ctx.task_max_sleep_ms = cfg->task_max_sleep_ms;
    if (lvgl_port_ctx.task_max_sleep_ms == 0) {
        lvgl_port_ctx.task_max_sleep_ms = 500;
    }
    lvgl_port_ctx.lvgl_mux = xSemaphoreCreateRecursiveMutex();
    ESP_GOTO_ON_FALSE(lvgl_port_ctx.lvgl_mux, ESP_ERR_NO_MEM, err, TAG, "Create LVGL mutex fail!");

    BaseType_t res;
    if (cfg->task_affinity < 0) {
        res = xTaskCreate(lvgl_port_task, "LVGL task", cfg->task_stack, NULL, cfg->task_priority, NULL);
    } else {
        res = xTaskCreatePinnedToCore(lvgl_port_task, "LVGL task", cfg->task_stack, NULL, cfg->task_priority, NULL, cfg->task_affinity);
    }
    ESP_GOTO_ON_FALSE(res == pdPASS, ESP_FAIL, err, TAG, "Create LVGL task fail!");

err:
    if (ret != ESP_OK) {
        lvgl_port_deinit();
    }

    return ret;
}

/*******************************************************************************
* Additional Functions for Waveshare 1.69" LCD Touch ESP32-S3R8
*******************************************************************************/

#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
static void lvgl_port_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    assert(indev_drv);
    lvgl_port_touch_ctx_t *touch_ctx = (lvgl_port_touch_ctx_t *)indev_drv->user_data;
    assert(touch_ctx->handle);

    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read data from touch controller into memory */
    esp_lcd_touch_read_data(touch_ctx->handle);
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_ctx->handle, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif
