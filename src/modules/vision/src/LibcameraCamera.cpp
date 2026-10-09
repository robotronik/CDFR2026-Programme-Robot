#include "vision/LibcameraCamera.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>

#include <sys/mman.h>
#include <unistd.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/formats.h>

#include "utils/logger.hpp"

// libcamera 0.3 renamed the 24-bit packed RGB/BGR format constants
// (RGB888/BGR888 became RGB24/BGR24). CMake defines the selector from the
// detected libcamera version.
#ifdef CAMERA_LIBCAMERA_FORMAT_RGB24
#  define CAMERA_FORMAT_RGB24 libcamera::formats::RGB24
#  define CAMERA_FORMAT_BGR24 libcamera::formats::BGR24
#else
#  define CAMERA_FORMAT_RGB24 libcamera::formats::RGB888
#  define CAMERA_FORMAT_BGR24 libcamera::formats::BGR888
#endif

namespace vision {

namespace {

// Bail out of the frame wait after this long so a stalled driver cannot hang
// the capture thread forever.
constexpr int kAcquireTimeoutMs = 1000;

// True when the format stores its components as separate planes.
bool isPlanarPixelFormat(const libcamera::PixelFormat& format) {
    switch (format) {
    case libcamera::formats::R8:
    case libcamera::formats::R10:
    case libcamera::formats::R12:
    case libcamera::formats::R16:
    case libcamera::formats::YUYV:
    case libcamera::formats::UYVY:
    case libcamera::formats::RGB565:
    case CAMERA_FORMAT_RGB24:
    case CAMERA_FORMAT_BGR24:
    case libcamera::formats::XRGB8888:
    case libcamera::formats::XBGR8888:
    case libcamera::formats::ARGB8888:
    case libcamera::formats::ABGR8888:
    case libcamera::formats::RGBA8888:
    case libcamera::formats::BGRA8888:
        return false;
    default:
        // NV12, YUV420, YUV422 (planar variants), YUV444...
        return true;
    }
}

} // namespace

LibcameraCamera::LibcameraCamera(const std::string& cameraId) : cameraId_(cameraId) {}

LibcameraCamera::~LibcameraCamera() {
    close();
}

void LibcameraCamera::onRequestCompleted(libcamera::Request* request) {
    if (request->status() == libcamera::Request::RequestCancelled) {
        return;
    }
    --pending_;
    // Keep the pipeline fed: hand the buffer back to the camera for the next
    // frame. The first available buffer is published to captureFrame().
    if (started_) {
        request->reuse(libcamera::Request::ReuseBuffers);
        if (camera_->queueRequest(request) == 0) {
            ++pending_;
        }
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

    // Select the requested device when its id/index matches, otherwise fall back
    // to the first camera: with a single CsiCameraProvider the ordering a device
    // index implies is not guaranteed.
    std::string selected;
    const auto cameras = manager_->cameras();
    if (!cameraId_.empty()) {
        selected = cameraId_;
    } else if (deviceIndex >= 0 && static_cast<std::size_t>(deviceIndex) < cameras.size()) {
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

    pending_ = 0;
    ready_ = nullptr;
    started_ = false;
    captureFailCount_ = 0;
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
    // Request 8-bit monochrome (R8) by default: the robot camera (OV9281) is a
    // monochrome sensor. Requesting color formats like NV12 causes the ISP to run
    // unwanted demosaicing and color processing, resulting in color artifacts.
    streamConfig.pixelFormat = libcamera::formats::R8;
    streamConfig.size.width = static_cast<unsigned int>(width);
    streamConfig.size.height = static_cast<unsigned int>(height);

    libcamera::CameraConfiguration::Status status = config->validate();
    if (status == libcamera::CameraConfiguration::Invalid) {
        LOG_WARNING("Libcamera - R8 format invalid for camera, falling back to default configuration");
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

    // configure() may replace the stream configuration, so read the negotiated
    // values back after the call.
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
    multiPlanar_ = isPlanarPixelFormat(pixelFormat_);

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
        requests_.push_back(std::move(request));
    }
    return true;
}

void LibcameraCamera::queueAll() {
    for (const std::unique_ptr<libcamera::Request>& request : requests_) {
        if (camera_->queueRequest(request.get()) < 0) {
            LOG_ERROR("Libcamera - failed to queue a capture request");
            continue;
        }
        ++pending_;
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
        if (captureFailCount_ == 0 || captureFailCount_ % 400 == 0) {
            LOG_WARNING("Libcamera - no frame delivered (", captureFailCount_ + 1, " failures)");
        }
        ++captureFailCount_;
        return false;
    }

    if (captureFailCount_ > 0) {
        LOG_GREEN_INFO("Libcamera - frames resumed after ", captureFailCount_, " failures");
        captureFailCount_ = 0;
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

    if (!multiPlanar_) {
        // Packed and single-plane formats expose all their bytes in the first
        // plane.
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
        return;
    }

    // Planar formats are not contiguous in one camera buffer: pack the planes
    // back-to-back so the decoder can read them as a single image.
    std::size_t totalLength = 0;
    for (const auto& plane : planes) {
        totalLength += static_cast<std::size_t>(plane.length);
    }
    buffer_.resize(totalLength);
    std::size_t offset = 0;
    const long pageSize = sysconf(_SC_PAGE_SIZE);
    for (const libcamera::FrameBuffer::Plane& plane : planes) {
        const std::size_t length = static_cast<std::size_t>(plane.length);
        if (offset + length > buffer_.size()) {
            return;
        }
        const off_t pageOffset = (plane.offset != libcamera::FrameBuffer::Plane::kInvalidOffset)
                                     ? (plane.offset & ~(pageSize - 1))
                                     : 0;
        const std::size_t offsetDiff = (plane.offset != libcamera::FrameBuffer::Plane::kInvalidOffset)
                                           ? (plane.offset - pageOffset)
                                           : 0;
        const std::size_t mapLength = length + offsetDiff;
        void* mapped = mmap(nullptr, mapLength, PROT_READ, MAP_SHARED, plane.fd.get(), pageOffset);
        if (mapped == MAP_FAILED) {
            LOG_WARNING("Libcamera - failed to map a frame buffer plane");
            return;
        }
        std::memcpy(buffer_.data() + offset, static_cast<const uint8_t*>(mapped) + offsetDiff, length);
        munmap(mapped, mapLength);
        offset += length;
    }
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
    pending_ = 0;
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
