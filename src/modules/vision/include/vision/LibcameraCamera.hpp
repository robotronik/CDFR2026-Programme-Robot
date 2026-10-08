#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <libcamera/base/object.h>
#include <libcamera/libcamera.h>

#include "vision/RawFrame.hpp"

namespace vision {

/**
 * libcamera capture backend.
 *
 * Owns a libcamera Camera and the whole capture pipeline (configuration,
 * buffer allocation, request queueing). The Raspberry Pi 5's OV9281 sits behind
 * the PiSP ISP, so the plain V4L2 node only exposes raw Bayer frames that
 * OpenCV cannot decode; libcamera drives the full sensor -> ISP -> frame
 * pipeline and hands back usable pixels.
 *
 * Deriving from libcamera::Object is required, not cosmetic: libcamera runs the
 * CameraManager (and therefore the pipeline that emits
 * Camera::requestCompleted) on its own internal thread. A plain C++ receiver is
 * called directly on that thread, so a receiver that blocks in
 * EventDispatcher::processEvents() on another thread never sees the signal.
 * As an Object, the connection is marshalled onto the thread that created this
 * object, which is the capture thread that pumps the dispatcher.
 *
 * The latest completed frame is copied into an owned buffer that stays valid
 * until the following capture, so callers keep the usual borrow-within-a-frame
 * contract of the capture API.
 */
class LibcameraCamera : public libcamera::Object {
public:
    explicit LibcameraCamera(const std::string& cameraId = {});
    ~LibcameraCamera();

    LibcameraCamera(const LibcameraCamera&) = delete;
    LibcameraCamera& operator=(const LibcameraCamera&) = delete;

    // Starts the camera manager, acquires a camera and configures it for the
    // requested size (clamped to the format the sensor can deliver).
    bool open(int deviceIndex, int width, int height);

    // Stops and releases everything acquired by open().
    void close();

    bool isOpen() const { return open_; }

    // Waits for the next frame (bounded by a timeout) and makes it available
    // through the last argument. Returns false on timeout, queue error or when
    // the camera is not open.
    bool captureFrame(RawFrame& out);

private:
    bool configure(int width, int height);
    bool allocateBuffers();
    void queueAll();
    void onRequestCompleted(libcamera::Request* request);
    void fillRawFrame(RawFrame& out) const;

    std::string cameraId_;
    std::unique_ptr<libcamera::CameraManager> manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    libcamera::Stream* stream_ = nullptr;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;

    std::size_t pending_ = 0;
    const libcamera::FrameBuffer* ready_ = nullptr;

    bool open_ = false;
    // libcamera 0.2 has no Camera::isRunning(), so the running state is tracked
    // here to only stop/queue once capture has actually started.
    bool started_ = false;
    unsigned int captureFailCount_ = 0;

    // Metrics of the configured stream, needed to interpret the completed
    // frame's planes into a RawFrame.
    int width_ = 0;
    int height_ = 0;
    int stride_ = 0;
    int pixelFormat_ = 0;
    // True when the format is stored as several planes and has to be flattened
    // before conversion.
    bool multiPlanar_ = false;

    // Scratch copy of the latest frame, repacked from the camera buffer so it
    // outlives the borrowed mmap'ed memory. Mutable because fillRawFrame() is
    // const (it only publishes through RawFrame).
    mutable std::vector<uint8_t> buffer_;
};

} // namespace vision
