#ifndef ESR_FONT_HPP
#define ESR_FONT_HPP

#include <stdint.h>
#include "esr/esr_raster.hpp"

namespace esr {

/**
 * Minimal 5x7 ASCII bitmap font (characters 32 ' ' to 126 '~').
 * 5 columns per character, 1 byte per column (bit 0 = top pixel, bit 6 = bottom pixel).
 */
extern const uint8_t ESR_FONT_5X7[95][5];

/**
 * Draw a single 5x7 ASCII character into the RGB888 framebuffer.
 *
 * @param fb Framebuffer target.
 * @param x Top-left pixel X.
 * @param y Top-left pixel Y.
 * @param c Character to draw (ASCII 32..126).
 * @param r Text Red [0..255].
 * @param g Text Green [0..255].
 * @param b Text Blue [0..255].
 * @param draw_bg If true, draws a solid dark background backing box for readability.
 */
inline void draw_char(Framebuffer& fb, int x, int y, char c,
                      uint8_t r = 255, uint8_t g = 255, uint8_t b = 255,
                      bool draw_bg = true, uint8_t bg_r = 16, uint8_t bg_g = 16, uint8_t bg_b = 16) {
    if (fb.color == nullptr || fb.width <= 0 || fb.height <= 0) return;
    if (c < 32 || c > 126) c = '?';

    const uint8_t* glyph = ESR_FONT_5X7[c - 32];

    for (int col = 0; col < 6; ++col) {
        int px = x + col;
        if (px < 0 || px >= fb.width) continue;

        uint8_t bits = (col < 5) ? glyph[col] : 0; // 6th column is 1px spacing

        for (int row = 0; row < 8; ++row) {
            int py = y + row;
            if (py < 0 || py >= fb.height) continue;

            uint8_t* p = fb.color + (py * fb.width + px) * 3;
            if (bits & (1 << row)) {
                p[0] = r;
                p[1] = g;
                p[2] = b;
            } else if (draw_bg) {
                p[0] = bg_r;
                p[1] = bg_g;
                p[2] = bg_b;
            }
        }
    }
}

/**
 * Draw a string of text into the RGB888 framebuffer using the 5x7 font.
 */
inline void draw_text(Framebuffer& fb, int x, int y, const char* str,
                      uint8_t r = 255, uint8_t g = 255, uint8_t b = 255,
                      bool draw_bg = true) {
    if (!str) return;
    int cur_x = x;
    int cur_y = y;

    while (*str) {
        if (*str == '\n') {
            cur_x = x;
            cur_y += 9; // Line height 8px + 1px spacing
        } else {
            draw_char(fb, cur_x, cur_y, *str, r, g, b, draw_bg);
            cur_x += 6; // Glyph width 5px + 1px spacing
        }
        ++str;
    }
}

} // namespace esr

#endif // ESR_FONT_HPP
