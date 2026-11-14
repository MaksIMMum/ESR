#include "esr/esr_depth.hpp"

namespace esr {

void clear_depth(DepthBuffer& db, uint16_t clear_val) {
    if (!db.data || db.width <= 0 || db.height <= 0) {
        return;
    }

    const size_t total_pixels = static_cast<size_t>(db.width) * db.height;
    if (clear_val == 0xFFFF) {
        // Fast hardware memset: 0xFF in every byte produces 0xFFFF for every uint16_t
        memset(db.data, 0xFF, total_pixels * sizeof(uint16_t));
    } else if (clear_val == 0x0000) {
        memset(db.data, 0x00, total_pixels * sizeof(uint16_t));
    } else {
        // Arbitrary clear value
        for (size_t i = 0; i < total_pixels; ++i) {
            db.data[i] = clear_val;
        }
    }
}

} // namespace esr
