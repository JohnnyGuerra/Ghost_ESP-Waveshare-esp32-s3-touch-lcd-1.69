#ifndef MAIN_MENU_SCREEN_H
#define MAIN_MENU_SCREEN_H

#include "lvgl.h"
#include "managers/display_manager.h"
#include "managers/views/options_screen.h"

#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
#include "esp_lcd_touch.h"
#endif

/**
 * @brief Creates the main menu screen view.
 */
void main_menu_create(void);

/**
 * @brief Destroys the main menu screen view.
 */
void main_menu_destroy(void);

/**
 * @brief Handles menu item selection.
 *
 * @param item_index Index of the selected menu item.
 */
static void handle_menu_item_selection(int item_index);

/**
 * @brief Handles hardware button presses.
 *
 * @param ButtonPressed The button that was pressed.
 */
void handle_hardware_button_press(int ButtonPressed);

/**
 * @brief Updates the time on the main menu screen.
 */
void update_time_on_main_menu(void);

#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
/**
 * @brief Handles touch input for the main menu.
 *
 * @param touch_data Touch input data.
 */
void handle_touch_input(const lv_indev_data_t *touch_data);
#endif

extern View main_menu_view;

extern lv_timer_t *time_update_timer;

#endif /* MAIN_MENU_SCREEN_H */