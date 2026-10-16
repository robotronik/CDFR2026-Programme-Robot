#include "vision/LibcameraCamera.hpp"

#include <chrono>
#include <cstring>

#include <sys/mman.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>

#include "utils/logger.hpp"

namespace vision {

namespace {

// Give up the frame wait after this long so a stalled driver cannot hang capture.
constexpr int kAcquireTimeoutMs = 1000;

} // namespace

LibcameraCamera::~LibcameraCamera() {
    close();
}

void LibcameraCamera::onRequestCompleted(libcamera::Request* request) {
    if (request->status() == libcamera::Request::RequestCancelled) {
        return;
    }
    // Requeue the buffer to keep the pipeline fed; publish the first available.
    if (started_) {
        request->reuse(libcamera::Request::ReuseBuffers);
        applyControls(request);
        camera_->queueRequest(request);
    }
    if (ready_ == nullptr) {
        ready_ = request->buffers().begin()->second;
    }
}

bool LibcameraCamera::open(int deviceIndex, int width, int height) {
    close();

    manager_ = std::make_unique<libcamera::CameraManager>();
    if (manager_->start() < 0) {
        LOG_ERROR("Libcamera - failed to start the camera manager");
        manager_.reset();
        return false;
    }

    if (manager_->cameras().empty()) {
        LOG_ERROR("Libcamera - no camera detected");
        manager_->stop();
        manager_.reset();
        return false;
    }

    // Select the requested device when the index is in range, else the first
    // camera (with a single CsiCameraProvider, a device index implies no
    // ordering).
    std::string selected;
    const auto cameras = manager_->cameras();
    if (deviceIndex >= 0 && static_cast<std::size_t>(deviceIndex) < cameras.size()) {
        selected = cameras[deviceIndex]->id();
    }
    if (selected.empty() || manager_->get(selected) == nullptr) {
        selected = cameras.front()->id();
    }

    camera_ = manager_->get(selected);
    if (!camera_ || camera_->acquire() < 0) {
        LOG_ERROR("Libcamera - failed to acquire camera '", selected, "' (busy?)");
        camera_.reset();
        manager_->stop();
        manager_.reset();
        return false;
    }

    if (!configure(width, height) || !allocateBuffers()) {
        camera_->release();
        camera_.reset();
        manager_->stop();
        manager_.reset();
        return false;
    }

    ready_ = nullptr;
    started_ = false;
    open_ = true;
    LOG_GREEN_INFO("Libcamera - camera '", selected, "' opened at ", width_, "x", height_,
                   " (", pixelFormat_.toString(), ")");
    return true;
}

bool LibcameraCamera::configure(int width, int height) {
    std::unique_ptr<libcamera::CameraConfiguration> config =
        camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
    if (!config) {
        LOG_ERROR("Libcamera - failed to generate a camera configuration");
        return false;
    }

    libcamera::StreamConfiguration& streamConfig = config->at(0);
    // YUV420 passes through the Pi 5's hardware ISP (contrast/brightness/
    // exposure); convertToGray keeps only the Y plane.
    streamConfig.pixelFormat = libcamera::formats::YUV420;
    streamConfig.size.width = static_cast<unsigned int>(width);
    streamConfig.size.height = static_cast<unsigned int>(height);

    libcamera::CameraConfiguration::Status status = config->validate();
    if (status == libcamera::CameraConfiguration::Invalid) {
        LOG_WARNING("Libcamera - YUV420 format invalid for camera, falling back to default configuration");
        config = camera_->generateConfiguration({libcamera::StreamRole::Viewfinder});
        if (!config) {
            LOG_ERROR("Libcamera - failed to generate a fallback camera configuration");
            return false;
        }
        libcamera::StreamConfiguration& fallbackStreamConfig = config->at(0);
        fallbackStreamConfig.size.width = static_cast<unsigned int>(width);
        fallbackStreamConfig.size.height = static_cast<unsigned int>(height);
        status = config->validate();
        if (status == libcamera::CameraConfiguration::Invalid) {
            LOG_ERROR("Libcamera - camera configuration is invalid");
            return false;
        }
    }
    if (status == libcamera::CameraConfiguration::Adjusted) {
        LOG_WARNING("Libcamera - requested ", width, "x", height, " adjusted to ",
                    config->at(0).size.width, "x", config->at(0).size.height, " (",
                    config->at(0).pixelFormat.toString(), ")");
    }

    if (camera_->configure(config.get()) < 0) {
        LOG_ERROR("Libcamera - failed to apply the camera configuration");
        return false;
    }

    // configure() may replace the configuration; read the negotiated values back.
    const libcamera::StreamConfiguration& negotiated = config->at(0);
    stream_ = negotiated.stream();
    if (stream_ == nullptr) {
        LOG_ERROR("Libcamera - the camera configuration has no stream");
        return false;
    }

    width_ = static_cast<int>(negotiated.size.width);
    height_ = static_cast<int>(negotiated.size.height);
    stride_ = static_cast<int>(negotiated.stride);
    pixelFormat_ = negotiated.pixelFormat;

    config_ = std::move(config);
    return true;
}

bool LibcameraCamera::allocateBuffers() {
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);
    if (allocator_->allocate(stream_) < 0) {
        LOG_ERROR("Libcamera - failed to allocate frame buffers");
        return false;
    }

    const std::vector<std::unique_ptr<libcamera::FrameBuffer>>& buffers = allocator_->buffers(stream_);
    if (buffers.empty()) {
        LOG_ERROR("Libcamera - no frame buffers were allocated");
        return false;
    }

    requests_.clear();
    for (const std::unique_ptr<libcamera::FrameBuffer>& buffer : buffers) {
        std::unique_ptr<libcamera::Request> request = camera_->createRequest();
        if (request == nullptr) {
            LOG_ERROR("Libcamera - failed to create a capture request");
            return false;
        }
        if (request->addBuffer(stream_, buffer.get()) < 0) {
            LOG_ERROR("Libcamera - failed to attach a buffer to a request");
            return false;
        }
        applyControls(request.get());
        requests_.push_back(std::move(request));
    }
    return true;
}

void LibcameraCamera::applyControls(libcamera::Request* request) const {
    if (request == nullptr) {
        return;
    }
    request->controls().set(libcamera::controls::ExposureValue, exposureValue_);
    request->controls().set(libcamera::controls::Contrast, contrast_);
    request->controls().set(libcamera::controls::Brightness, brightness_);
}

void LibcameraCamera::setExposureValue(float ev) {
    exposureValue_ = ev;
}

void LibcameraCamera::setContrast(float contrast) {
    contrast_ = contrast;
}

void LibcameraCamera::setBrightness(float brightness) {
    brightness_ = brightness;
}

void LibcameraCamera::queueAll() {
    for (const std::unique_ptr<libcamera::Request>& request : requests_) {
        if (camera_->queueRequest(request.get()) < 0) {
            LOG_ERROR("Libcamera - failed to queue a capture request");
        }
    }
}

bool LibcameraCamera::captureFrame(RawFrame& out) {
    if (!open_) {
        return false;
    }

    if (!started_) {
        if (camera_->start() < 0) {
            LOG_ERROR("Libcamera - failed to start the camera");
            return false;
        }
        started_ = true;
        camera_->requestCompleted.connect(this, &LibcameraCamera::onRequestCompleted);
        queueAll();
    }

    // Drain the dispatcher until a frame arrives or the timeout expires.
    ready_ = nullptr;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(kAcquireTimeoutMs);
    libcamera::EventDispatcher* dispatcher =
        libcamera::Thread::current()->eventDispatcher();
    while (ready_ == nullptr && dispatcher != nullptr) {
        dispatcher->processEvents();
        if (ready_ == nullptr && std::chrono::steady_clock::now() >= deadline) {
            break;
        }
    }

    if (ready_ == nullptr) {
        // Reporting a missing frame is ArucoDetector's job, for every backend.
        return false;
    }

    fillRawFrame(out);
    ready_ = nullptr;
    return true;
}

void LibcameraCamera::fillRawFrame(RawFrame& out) const {
    out.width = width_;
    out.height = height_;
    out.stride = stride_;
    out.format = pixelFormat_;
    out.data = nullptr;

    // libcamera returned a std::vector<Plane> up to 0.6 and a Span<const Plane>
    // from 0.7; `auto` keeps both working.
    const auto& planes = ready_->planes();
    if (planes.empty()) {
        return;
    }

    // Publish plane 0 only: for a planar format it is the Y plane, which carries
    // all the luminance the grayscale pipeline uses, and a packed format keeps
    // every byte of the image in its first plane.
    const libcamera::FrameBuffer::Plane& plane = planes.front();
    const std::size_t length = static_cast<std::size_t>(plane.length);
    void* mapped = mmap(nullptr, length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (mapped == MAP_FAILED) {
        LOG_WARNING("Libcamera - failed to map the frame buffer");
        return;
    }
    buffer_.resize(length);
    std::memcpy(buffer_.data(), mapped, length);
    munmap(mapped, length);
    out.data = buffer_.data();
}

void LibcameraCamera::close() {
    if (!open_ && !camera_ && !manager_) {
        return;
    }

    if (camera_ && started_) {
        camera_->requestCompleted.disconnect(this, &LibcameraCamera::onRequestCompleted);
        camera_->stop();
        started_ = false;
    }

    // Requests are owned by the vector and destroyed here; the frame buffers
    // they reference are freed with the allocator below.
    requests_.clear();
    ready_ = nullptr;

    allocator_.reset();
    stream_ = nullptr;
    config_.reset();
    buffer_.clear();
    pixelFormat_ = libcamera::PixelFormat();

    if (camera_) {
        camera_->release();
        camera_.reset();
    }
    if (manager_) {
        manager_->stop();
        manager_.reset();
    }

    open_ = false;
}

} // namespace vision
