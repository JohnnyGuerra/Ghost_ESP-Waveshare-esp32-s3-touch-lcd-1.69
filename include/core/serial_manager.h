// serial_manager.h

#ifndef SERIAL_MANAGER_H
#define SERIAL_MANAGER_H

#include <esp_types.h>
#include <managers/display_manager.h>
#include <freertos/queue.h>

// Initialize the SerialManager
void serial_manager_init();

// Task function for reading serial commands
void serial_task(void *pvParameter);

// Handle a serial command
int handle_serial_command(const char *input);

// Simulate a serial command (for testing purposes)
void simulateCommand(const char *commandString);

// Queue for serial commands
extern QueueHandle_t commandQueue;

// Structure for serial commands
typedef struct {
  char command[1024];
} SerialCommand;

// Waveshare 1.69" LCD Touch ESP32-S3R8-specific serial commands
#ifdef CONFIG_WAVESHARE_169_LCD_TOUCH
void cmd_display_on_serial(const char *args);      // Command to turn on the display
void cmd_display_off_serial(const char *args);     // Command to turn off the display
void cmd_set_brightness_serial(const char *args);  // Command to set display brightness
void cmd_touch_status_serial(const char *args);    // Command to check touch controller status
#endif

#endif // SERIAL_MANAGER_H
