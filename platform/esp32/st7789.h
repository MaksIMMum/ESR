#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define ST7789_WIDTH   240
#define ST7789_HEIGHT  320

#define ST7789_PIN_MOSI 11
#define ST7789_PIN_SCLK 12
#define ST7789_PIN_CS   10
#define ST7789_PIN_DC   13
#define ST7789_PIN_RST  14

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SPI bus, GPIOs, and ST7789 display controller
 */
esp_err_t st7789_init(void);

/**
 * @brief Fill the entire 240x320 display with a single RGB565 color
 */
void st7789_fill_color(uint16_t rgb565);

/**
 * @brief Push a rectangular RGB565 pixel buffer to the display
 */
void st7789_draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);

/**
 * @brief Convert and draw an RGB888 framebuffer (like the current ESR triangle) onto the screen
 */
void st7789_draw_rgb888_buffer(int dst_x, int dst_y, int w, int h, const uint8_t *rgb888_data);

#ifdef __cplusplus
}
#endif
