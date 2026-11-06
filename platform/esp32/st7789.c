#include "st7789.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "ST7789";

static spi_device_handle_t s_spi = NULL;

static void lcd_cmd(uint8_t cmd)
{
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_data(const uint8_t *data, size_t len)
{
    if (len == 0) return;
    gpio_set_level((gpio_num_t)ST7789_PIN_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_polling_transmit(s_spi, &t);
}

static void lcd_byte(uint8_t b)
{
    lcd_data(&b, 1);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t d[4];

    lcd_cmd(0x2A); // CASET
    d[0] = (x0 >> 8) & 0xFF;
    d[1] = x0 & 0xFF;
    d[2] = (x1 >> 8) & 0xFF;
    d[3] = x1 & 0xFF;
    lcd_data(d, 4);

    lcd_cmd(0x2B); // RASET
    d[0] = (y0 >> 8) & 0xFF;
    d[1] = y0 & 0xFF;
    d[2] = (y1 >> 8) & 0xFF;
    d[3] = y1 & 0xFF;
    lcd_data(d, 4);

    lcd_cmd(0x2C); // RAMWR
}

esp_err_t st7789_init(void)
{
    ESP_LOGI(TAG, "Initializing ST7789 SPI on MOSI:%d SCLK:%d CS:%d DC:%d RST:%d",
             ST7789_PIN_MOSI, ST7789_PIN_SCLK, ST7789_PIN_CS, ST7789_PIN_DC, ST7789_PIN_RST);

    // 1. Configure DC and RST GPIOs
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ST7789_PIN_DC) | (1ULL << ST7789_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 2. Hardware reset
    gpio_set_level((gpio_num_t)ST7789_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level((gpio_num_t)ST7789_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 3. Configure SPI Bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = ST7789_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = ST7789_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ST7789_WIDTH * 32 * sizeof(uint16_t),
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 4. Attach ST7789 SPI Device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 40 * 1000 * 1000, // 40 MHz SPI clock
        .mode = 0,                          // SPI mode 0
        .spics_io_num = ST7789_PIN_CS,
        .queue_size = 7,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 5. ST7789 Initialization Sequence
    lcd_cmd(0x01); // Software Reset
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_cmd(0x11); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_cmd(0x3A); // Interface Pixel Format: 16-bit/pixel (RGB565)
    lcd_byte(0x55);

    lcd_cmd(0x36); // Memory Data Access Control (MADCTL)
    lcd_byte(0x00); // Default portrait RGB order

    lcd_cmd(0x21); // Display Inversion ON (required on most ST7789 panels)
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_cmd(0x13); // Normal Display Mode On
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_cmd(0x29); // Display On
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "ST7789 initialized successfully (%dx%d)", ST7789_WIDTH, ST7789_HEIGHT);
    return ESP_OK;
}

void st7789_fill_color(uint16_t rgb565)
{
    if (!s_spi) return;

    // Swap bytes for SPI big-endian transmission
    uint16_t color_be = (rgb565 >> 8) | (rgb565 << 8);

    const int chunk_lines = 16;
    size_t chunk_pixels = ST7789_WIDTH * chunk_lines;
    uint16_t *buf = heap_caps_malloc(chunk_pixels * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA chunk buffer");
        return;
    }

    for (size_t i = 0; i < chunk_pixels; ++i) {
        buf[i] = color_be;
    }

    lcd_set_window(0, 0, ST7789_WIDTH - 1, ST7789_HEIGHT - 1);

    for (int y = 0; y < ST7789_HEIGHT; y += chunk_lines) {
        int lines = (y + chunk_lines <= ST7789_HEIGHT) ? chunk_lines : (ST7789_HEIGHT - y);
        lcd_data((const uint8_t *)buf, ST7789_WIDTH * lines * sizeof(uint16_t));
    }

    free(buf);
}

void st7789_draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
    if (!s_spi || !data || w == 0 || h == 0) return;

    lcd_set_window(x, y, x + w - 1, y + h - 1);

    // Allocate DMA-capable chunk buffer (16 lines at a time)
    const int chunk_lines = 16;
    size_t chunk_pixels = w * chunk_lines;
    uint16_t *dma_buf = heap_caps_malloc(chunk_pixels * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!dma_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer for draw_bitmap");
        return;
    }

    size_t total_pixels = (size_t)w * h;
    size_t pixels_sent = 0;

    while (pixels_sent < total_pixels) {
        size_t to_send = (total_pixels - pixels_sent < chunk_pixels) ? (total_pixels - pixels_sent) : chunk_pixels;
        // Copy chunk into SRAM DMA buffer (allows data to reside in Flash / RODATA)
        memcpy(dma_buf, data + pixels_sent, to_send * sizeof(uint16_t));
        lcd_data((const uint8_t *)dma_buf, to_send * sizeof(uint16_t));
        pixels_sent += to_send;
    }

    free(dma_buf);
}

void st7789_draw_rgb888_buffer(int dst_x, int dst_y, int w, int h, const uint8_t *rgb888_data)
{
    if (!s_spi || !rgb888_data || w <= 0 || h <= 0) return;

    // Allocate 1-line DMA-capable buffer
    uint16_t *line_buf = heap_caps_malloc(w * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!line_buf) {
        ESP_LOGE(TAG, "Failed to allocate line buffer for blit");
        return;
    }

    lcd_set_window(dst_x, dst_y, dst_x + w - 1, dst_y + h - 1);

    for (int row = 0; row < h; ++row) {
        const uint8_t *src_row = rgb888_data + (row * w * 3);
        for (int col = 0; col < w; ++col) {
            uint8_t r = src_row[col * 3 + 0];
            uint8_t g = src_row[col * 3 + 1];
            uint8_t b = src_row[col * 3 + 2];

            // Convert RGB888 -> RGB565 and byte-swap for SPI
            uint16_t rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
            line_buf[col] = (rgb565 >> 8) | (rgb565 << 8);
        }
        lcd_data((const uint8_t *)line_buf, w * sizeof(uint16_t));
    }

    free(line_buf);
}
