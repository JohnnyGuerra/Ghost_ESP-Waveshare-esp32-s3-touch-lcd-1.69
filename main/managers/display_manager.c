#include "managers/display_manager.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl_helpers.h"
#include "managers/sd_card_manager.h"
#include "managers/settings_manager.h"
#include "managers/views/error_popup.h"
#include "managers/views/main_menu_screen.h"
#include "managers/views/options_screen.h"
#include "managers/views/terminal_screen.h"
#include <stdlib.h>

// --- Include drivers based on Kconfig ---
#ifdef CONFIG_USE_CARDPUTER
#include "vendor/keyboard_handler.h"
#include "vendor/m5/m5gfx_wrapper.h"
#endif

#ifdef CONFIG_HAS_BATTERY
    #ifdef CONFIG_ESP32S3R8_HAS_BATTERY_ETA6098
        #include "driver/eta6098.h"
    #endif
#endif

#ifdef CONFIG_HAS_RTC_CLOCK
    #ifdef CONFIG_ESP32S3R8_HAS_RTC_PCF85063
        #include "driver/pcf85063.h"
    #endif
#endif

#ifdef CONFIG_ESP32S3R8_HAS_IMU_QMI8658
#include "driver/qmi8658.h"
#endif

#ifdef CONFIG_ESP32S3R8_HAS_TOUCH
#include "driver/cst816.h" // Replace with the actual touch driver
#endif

#ifdef CONFIG_USE_169_INCHER
#include "vendor/drivers/st7789.h"
#endif

#define LVGL_TASK_PERIOD_MS 5
static const char *TAG = "DisplayManager";
DisplayManager dm = {.current_view = NULL, .previous_view = NULL};

lv_obj_t *status_bar;
lv_obj_t *wifi_label = NULL;
lv_obj_t *bt_label = NULL;
lv_obj_t *sd_label = NULL;
lv_obj_t *battery_label = NULL;

bool display_manager_init_success = false;

#define FADE_DURATION_MS 10
#define DEFAULT_DISPLAY_TIMEOUT_MS 10000
uint32_t display_timeout_ms = DEFAULT_DISPLAY_TIMEOUT_MS;

// --- Forward Declarations ---
static void lvgl_tick_task(void *pvParameters);
static void hardware_input_task(void *pvParameters);
static void set_backlight_brightness(uint8_t percentage);

// --- I2C Initialization ---
static void i2c_master_init(void) {
    ESP_LOGI(TAG, "Initializing I2C Master for ESP32-S3R8 peripherals");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_ESP32S3R8_PIN_NUM_TOUCH_SDA,
        .scl_io_num = CONFIG_ESP32S3R8_PIN_NUM_TOUCH_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // 400kHz
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

// --- Display Manager Initialization ---
void display_manager_init(void) {
    lv_init();

    // Initialize SPI for LCD
    spi_bus_config_t buscfg = {
        .mosi_io_num = CONFIG_ESP32S3R8_PIN_NUM_DISPLAY_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = CONFIG_ESP32S3R8_PIN_NUM_DISPLAY_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CONFIG_TFT_WIDTH * CONFIG_TFT_HEIGHT * 2 + 8
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // Initialize display driver
    disp_spi_initialize(SPI2_HOST,
                        CONFIG_ESP32S3R8_PIN_NUM_DISPLAY_CS,
                        CONFIG_ESP32S3R8_PIN_NUM_DISPLAY_DC,
                        CONFIG_ESP32S3R8_PIN_NUM_DISPLAY_RST);
    disp_driver_init();

    // Initialize I2C for peripherals
    i2c_master_init();

    // Initialize touch controller
    #ifdef CONFIG_ESP32S3R8_HAS_TOUCH
    ESP_LOGI(TAG, "Initializing Touch Controller");
    touch_driver_init();
    #endif

    // Initialize RTC
    #ifdef CONFIG_ESP32S3R8_HAS_RTC_PCF85063
    ESP_LOGI(TAG, "Initializing PCF85063 RTC");
    pcf85063_init(I2C_NUM_0, CONFIG_ESP32S3R8_RTC_I2C_ADDR);
    #endif

    // Initialize IMU
    #ifdef CONFIG_ESP32S3R8_HAS_IMU_QMI8658
    ESP_LOGI(TAG, "Initializing QMI8658 IMU");
    qmi8658_init(I2C_NUM_0, QMI8658_I2C_ADDRESS);
    #endif

    // Initialize Battery Manager
    #ifdef CONFIG_ESP32S3R8_HAS_BATTERY_ETA6098
    ESP_LOGI(TAG, "Initializing ETA6098 Battery Manager");
    eta6098_init();
    #endif

    // Initialize Backlight
    set_backlight_brightness(100);

    display_manager_init_success = true;

    // Start LVGL tasks
    xTaskCreate(lvgl_tick_task, "LVGL Tick Task", 4096, NULL, 5, NULL);
    xTaskCreate(hardware_input_task, "Hardware Input Task", 4096, NULL, 5, NULL);
}

// --- Backlight Control ---
void set_backlight_brightness(uint8_t percentage) {
    int bckl_pin = CONFIG_ESP32S3R8_PIN_NUM_DISPLAY_BCKL;
    if (bckl_pin >= 0) {
        gpio_set_level(bckl_pin, percentage > 0 ? 1 : 0);
    }
}

// --- LVGL Tick Task ---
static void lvgl_tick_task(void *pvParameters) {
    while (1) {
        lv_tick_inc(LVGL_TASK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_PERIOD_MS));
    }
    vTaskDelete(NULL);
}

// --- Hardware Input Task ---
static void hardware_input_task(void *pvParameters) {
    while (1) {
        // Handle input events (e.g., touch, buttons)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}
