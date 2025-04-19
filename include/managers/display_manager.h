#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "lvgl.h"
#include "managers/joystick_manager.h"
#include <stdbool.h>

#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
#include "esp_lcd_touch.h"
#endif

typedef void *QueueHandle_tt;
typedef void *SemaphoreHandle_tt; // Because Circular Includes are fun :)

typedef enum { INPUT_TYPE_JOYSTICK, INPUT_TYPE_TOUCH } InputType;

typedef struct {
  InputType type;
  union {
    int joystick_index;         // Used for joystick inputs
    lv_indev_data_t touch_data; // Used for touchscreen inputs
  } data;
} InputEvent;

#define INPUT_QUEUE_LENGTH 10
#define INPUT_ITEM_SIZE sizeof(int)
QueueHandle_tt input_queue;

#define MUTEX_TIMEOUT_MS 100

#define HARDWARE_INPUT_TASK_PRIORITY (4)
#define RENDERING_TASK_PRIORITY (4)

typedef struct {
  lv_obj_t *root;
  void (*create)(void);
  void (*destroy)(void);
  const char *name;
  void (*get_hardwareinput_callback)(void **callback);
  void (*input_callback)(InputEvent *);
} View;

typedef struct {
  View *current_view;
  View *previous_view;
  SemaphoreHandle_tt mutex;
} DisplayManager;

/* Function prototypes */

/**
 * @brief Initialize the Display Manager.
 */
void display_manager_init(void);

/**
 * @brief Register a new view.
 */
bool display_manager_register_view(View *view);

/**
 * @brief Switch to a new view.
 */
void display_manager_switch_view(View *view);

/**
 * @brief Destroy the current view.
 */
void display_manager_destroy_current_view(void);

/**
 * @brief Get the current active view.
 */
View *display_manager_get_current_view(void);

/**
 * @brief LVGL tick task for periodic updates.
 */
void lvgl_tick_task(void *arg);

/**
 * @brief Task to handle hardware input.
 */
void hardware_input_task(void *pvParameters);

/**
 * @brief Fill the screen with a specific color.
 *
 * @param color The color to fill the screen with.
 */
void display_manager_fill_screen(lv_color_t color);

/**
 * @brief Convert a hex color string to an LVGL color.
 *
 * @param hex_str The hex color string (e.g., "#FFFFFF").
 * @return The corresponding LVGL color.
 */
lv_color_t hex_to_lv_color(const char *hex_str);

/**
 * @brief Update the status bar with system information.
 *
 * @param wifi_enabled Whether Wi-Fi is enabled.
 * @param bt_enabled Whether Bluetooth is enabled.
 * @param sd_card_mounted Whether the SD card is mounted.
 * @param batteryPercentage The current battery percentage.
 */
void update_status_bar(bool wifi_enabled, bool bt_enabled, bool sd_card_mounted,
                       int batteryPercentage);

/**
 * @brief Add a status bar to the display.
 *
 * @param CurrentMenuName The name of the current menu to display in the status bar.
 */
void display_manager_add_status_bar(const char *CurrentMenuName);

#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
/**
 * @brief Initialize the touch controller for the Waveshare board.
 */
void display_manager_init_touch(void);

/**
 * @brief Handle touch input events.
 *
 * @param touch_data The touch input data.
 */
void display_manager_handle_touch_input(const lv_indev_data_t *touch_data);
#endif

// Declare images for use in the display
LV_IMG_DECLARE(Ghost_ESP);
LV_IMG_DECLARE(Map);
LV_IMG_DECLARE(bluetooth);
LV_IMG_DECLARE(wifi);
LV_IMG_DECLARE(rave);
LV_IMG_DECLARE(GESPFlappyghost);
LV_IMG_DECLARE(ghost);
LV_IMG_DECLARE(GESPAppGallery);

// Joystick array
joystick_t joysticks[5];

#endif /* DISPLAY_MANAGER_H */