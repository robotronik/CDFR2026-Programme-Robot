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

// libcamera capture backend: owns the camera and the whole capture pipeline
// (configuration, buffers, request queue). On the Pi 5 the OV9281 sits behind the
// PiSP ISP, so the plain V4L2 node only exposes raw Bayer frames OpenCV cannot
// decode; libcamera drives the sensor -> ISP -> frame pipeline.
//
// Deriving from libcamera::Object is required: libcamera delivers
// requestCompleted on its own internal thread, and only an Object's signal
// connection is marshalled onto the thread that created it (the capture thread
// that pumps the dispatcher). The latest frame is copied into an owned buffer
// that stays valid until the next capture.
class LibcameraCamera : public libcamera::Object {
public:
    explicit LibcameraCamera(const std::string& cameraId = {});
    ~LibcameraCamera();

    LibcameraCamera(const LibcameraCamera&) = delete;
    LibcameraCamera& operator=(const LibcameraCamera&) = delete;

    // Starts the camera manager, acquires a camera and configures it for the
    // requested size (clamped to what the sensor can deliver).
    bool open(int deviceIndex, int width, int height);

    // Stops and releases everything acquired by open().
    void close();

    bool isOpen() const { return open_; }

    // Controls
    void setExposureValue(float ev);
    void setContrast(float contrast);
    void setBrightness(float brightness);
    float getExposureValue() const { return exposureValue_; }
    float getContrast() const { return contrast_; }
    float getBrightness() const { return brightness_; }

    // Waits for the next frame (bounded by a timeout) into `out`. False on
    // timeout or error.
    bool captureFrame(RawFrame& out);

private:
    bool configure(int width, int height);
    bool allocateBuffers();
    void queueAll();
    void onRequestCompleted(libcamera::Request* request);
    void fillRawFrame(RawFrame& out) const;
    void applyControls(libcamera::Request* request) const;

    float exposureValue_ = 1.5f;
    float contrast_ = 1.3f;
    float brightness_ = 0.0f;

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
    // libcamera 0.2 has no Camera::isRunning(); track it to stop/queue only once
    // capture has started.
    bool started_ = false;
    unsigned int captureFailCount_ = 0;

    // Configured stream metrics, to interpret the completed frame's planes.
    int width_ = 0;
    int height_ = 0;
    int stride_ = 0;
    libcamera::PixelFormat pixelFormat_;
    // True when the format is stored as several planes to be flattened.
    bool multiPlanar_ = false;

    // Owned copy of the latest frame, repacked so it outlives the mmap'ed camera
    // buffer. Mutable because fillRawFrame() is const.
    mutable std::vector<uint8_t> buffer_;
};

} // namespace vision
