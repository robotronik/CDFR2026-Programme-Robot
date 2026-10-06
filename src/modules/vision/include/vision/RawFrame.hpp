#pragma once

#include <cstdint>

namespace vision {

/**
 * Non-owning, immutable view of one captured frame.
 *
 * A backend fills this in when it delivers a frame: `data` points to packed
 * pixels, `stride` is the number of bytes per row and the layout still has to
 * be converted to BGR before use. The buffer is only valid until the next
 * `captureFrame()` call, so callers must copy what they keep.
 */
struct RawFrame {
    int width = 0;
    int height = 0;
    int stride = 0;
    int format = 0;
    const uint8_t* data = nullptr;
};

} // namespace vision
