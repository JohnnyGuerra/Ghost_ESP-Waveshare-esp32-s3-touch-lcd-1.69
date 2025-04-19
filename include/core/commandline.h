// command.h

#ifndef COMMAND_H
#define COMMAND_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "esp_lcd_touch.h"

// Define the function pointer type for commands
typedef void (*CommandFunction)(int argc, char **argv);

// Command structure
typedef struct Command {
  char *name;
  CommandFunction function;
  struct Command *next;
} Command;

// Functions to manage commands
void command_init();
void register_command(const char *name, CommandFunction function);
void unregister_command(const char *name);
CommandFunction find_command(const char *name);

// Task handle for visualizer
extern TaskHandle_t VisualizerHandle;

// Function to register all commands
void register_commands();

// Waveshare 1.69" LCD Touch ESP32-S3R8-specific commands
#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
void cmd_display_on(int argc, char **argv);      // Command to turn on the display
void cmd_display_off(int argc, char **argv);     // Command to turn off the display
void cmd_set_brightness(int argc, char **argv);  // Command to set display brightness
void cmd_touch_status(int argc, char **argv);    // Command to check touch controller status
#endif

#endif // COMMAND_H