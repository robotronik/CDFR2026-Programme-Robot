#pragma once

#include <cstdint>

namespace vision {

// Non-owning view of one captured frame: `data` points to packed pixels with
// `stride` bytes per row. The layout is backend-specific and must be converted
// to BGR. The buffer is only valid until the next capture.
struct RawFrame {
    int width = 0;
    int height = 0;
    int stride = 0;
    int format = 0;
    const uint8_t* data = nullptr;
};

} // namespace vision
