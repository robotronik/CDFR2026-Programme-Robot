#include "vision/FeaturesLocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/logger.hpp"

namespace vision {

namespace {

// Working grid. The map and the rectified patch are both resampled to this
// millimetre-per-pixel value, so a match between them is a rigid motion with no
// scale term. 4 mm/px is the knee of the sweep in features.py: the fastest grid
// that still solves essentially everything.
constexpr double kMmPerPx = 4.0;

// The grid the pixel thresholds below were measured at. Those thresholds are
// rescaled with the working grid (a coarser grid yields fewer keypoints).
constexpr double kReferenceMmPerPx = 2.0;

// AKAZE with binary MLDB descriptors. The size is required at its full 486-bit
// width, not a tuning knob: OpenCV silently returns a 1-byte descriptor
// otherwise.
constexpr float kAkazeThreshold = 0.001f;
constexpr int kAkazeDescriptorSize = 486;
constexpr int kAkazeDescriptorChannels = 3;

// Lowe's ratio test, raised for AKAZE's more selective descriptors.
constexpr float kRatioTest = 0.85f;

// RANSAC fit of the map-to-patch rigid motion.
constexpr double kRansacMm = 6.0;
constexpr double kRansacConfidence = 0.999;
constexpr int kRansacMaxIters = 5000;
constexpr int kRansacRefineIters = 10;
constexpr double kMinInliers = 25.0;
constexpr double kMinInlierSpanMm = 300.0;

// Odometry prior slack: how far the true pose may be from the prior. The window
// sweeps the patch footprint through this yaw range.
constexpr double kPriorYawSlackDeg = 15.0;

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle <= -180.0) angle += 360.0;
    return angle;
}

// Inlier floor for the working grid, following `configure_scale` in features.py.
int activeMinInliers() {
    const double ratio = (kReferenceMmPerPx / kMmPerPx) * (kReferenceMmPerPx / kMmPerPx);
    return std::max(6, static_cast<int>(std::lround(kMinInliers * ratio)));
}

// Camera-to-ground rotation with columns (right, up, forward), looking along -Z
// and pitched down. Same construction as common/geometry.camera_rotation.
cv::Matx33d cameraRotation(double pitchDeg) {
    const double p = pitchDeg * kDegToRad;
    const double cp = std::cos(p);
    const double sp = std::sin(p);
    return cv::Matx33d(
         1.0,  0.0,  0.0,
         0.0,   cp,  -sp,
         0.0,  -sp,  -cp);
}

// Pixel -> ground millimetre homography, from the camera intrinsics and the
// mounting. Generalises common/geometry.capture_homography to any lens: it
// reduces to the same matrix when the principal point is centred and fx == fy.
cv::Matx33d captureHomography(const cv::Matx33d& cameraMatrix,
                              double pitchDeg, double heightMm) {
    cv::Matx33d kInv = cameraMatrix.inv();
    // Negate the vertical row so image rows, which grow downward, stay
    // consistent with the camera's up axis.
    kInv(1, 0) = -kInv(1, 0);
    kInv(1, 1) = -kInv(1, 1);
    kInv(1, 2) = -kInv(1, 2);
    const cv::Matx33d m = cameraRotation(pitchDeg) * kInv;

    const double camY = heightMm / 1000.0;
    const cv::Matx33d h(
        -camY * m(0, 0), -camY * m(0, 1), -camY * m(0, 2),
        -camY * m(2, 0), -camY * m(2, 1), -camY * m(2, 2),
              m(1, 0),        m(1, 1),        m(1, 2));
    const cv::Matx33d to_mm(1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 1.0);
    return to_mm * h;
}

// Ground area an image covers, as (x0, y0, x1, y1) millimetres. Sampled on a
// grid rather than at the corners: rows that land behind the camera are thrown
// far off to the sides. The third homogeneous coordinate is the depth and a
// real ground hit needs it negative.
void groundBounds(const cv::Matx33d& H, int width, int height,
                  double& x0, double& y0, double& x1, double& y1) {
    constexpr int kSamples = 65;
    x0 = y0 = 1e18;
    x1 = y1 = -1e18;
    bool any = false;
    for (int iy = 0; iy < kSamples; ++iy) {
        const double v = (height - 1) * iy / double(kSamples - 1);
        for (int ix = 0; ix < kSamples; ++ix) {
            const double u = (width - 1) * ix / double(kSamples - 1);
            const cv::Vec3d p = H * cv::Vec3d(u, v, 1.0);
            if (p[2] >= -1e-9) {
                continue;
            }
            const double gx = p[0] / p[2];
            const double gy = p[1] / p[2];
            x0 = std::min(x0, gx);
            x1 = std::max(x1, gx);
            y0 = std::min(y0, gy);
            y1 = std::max(y1, gy);
            any = true;
        }
    }
    if (!any) {
        x0 = y0 = 0.0;
        x1 = y1 = 1.0;
    }
}

// Map pixels -> field millimetres: x right, y up, origin at the field centre.
cv::Point2d mapPxToFieldMm(const cv::Point2d& px, const cv::Point2d& originPx,
                           double mmPerPx) {
    return cv::Point2d((px.x - originPx.x) * mmPerPx,
                       (originPx.y - px.y) * mmPerPx);
}

// Mutually-best matches that also pass the ratio test. Returns index arrays
// into the two descriptor sets.
void matchFeatures(const cv::Mat& queryDesc, const cv::Mat& trainDesc,
                   cv::BFMatcher& matcher, float ratio,
                   std::vector<int>& queryIdx, std::vector<int>& trainIdx) {
    queryIdx.clear();
    trainIdx.clear();
    if (queryDesc.rows < 2 || trainDesc.rows < 2) {
        return;
    }

    auto oneWay = [&](const cv::Mat& src, const cv::Mat& dst) {
        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(src, dst, knn, 2);
        std::vector<int> kept(src.rows, -1);
        for (const std::vector<cv::DMatch>& pair : knn) {
            if (pair.size() < 2) {
                continue;
            }
            if (pair[0].distance < ratio * pair[1].distance) {
                kept[pair[0].queryIdx] = pair[0].trainIdx;
            }
        }
        return kept;
    };

    const std::vector<int> forward = oneWay(queryDesc, trainDesc);
    const std::vector<int> backward = oneWay(trainDesc, queryDesc);
    for (int q = 0; q < static_cast<int>(forward.size()); ++q) {
        const int t = forward[q];
        if (t < 0 || t >= static_cast<int>(backward.size())) {
            continue;
        }
        if (backward[t] == q) {
            queryIdx.push_back(q);
            trainIdx.push_back(t);
        }
    }
}

// RANSAC fit of train = R(theta) * query + t with the scale fixed at 1. Returns
// false when the fit is too weak or drifts off unit scale.
bool solveRigid(const std::vector<cv::Point2d>& query,
                const std::vector<cv::Point2d>& train,
                cv::Mat& transform, cv::Mat& inlierMask) {
    const int minInliers = activeMinInliers();
    if (static_cast<int>(query.size()) < minInliers ||
        query.size() != train.size()) {
        return false;
    }

    cv::Mat from(static_cast<int>(query.size()), 2, CV_32F);
    cv::Mat to(static_cast<int>(train.size()), 2, CV_32F);
    for (int i = 0; i < static_cast<int>(query.size()); ++i) {
        from.at<float>(i, 0) = static_cast<float>(query[i].x);
        from.at<float>(i, 1) = static_cast<float>(query[i].y);
        to.at<float>(i, 0) = static_cast<float>(train[i].x);
        to.at<float>(i, 1) = static_cast<float>(train[i].y);
    }

    transform = cv::estimateAffinePartial2D(
        from, to, inlierMask, cv::RANSAC, kRansacMm / kMmPerPx,
        kRansacMaxIters, kRansacConfidence, kRansacRefineIters);
    if (transform.empty() || inlierMask.empty()) {
        return false;
    }

    const uchar* mask = inlierMask.ptr<uchar>(0);
    int count = 0;
    for (int i = 0; i < inlierMask.rows; ++i) {
        count += mask[i] != 0;
    }
    if (count < minInliers) {
        return false;
    }

    // The warp fixes the scale at exactly 1; a wrong-but-self-consistent scale
    // could otherwise pass RANSAC by shrinking the patch onto a similar region.
    const double scale = std::hypot(transform.at<double>(0, 0),
                                    transform.at<double>(1, 0));
    if (scale < 0.9 || scale > 1.1) {
        return false;
    }

    // A tight cluster of inliers is how a repeated texture passes everything
    // else; a real fix has matches spread across the patch it covers.
    double minX = 1e18, minY = 1e18, maxX = -1e18, maxY = -1e18;
    for (int i = 0; i < inlierMask.rows; ++i) {
        if (!mask[i]) {
            continue;
        }
        minX = std::min(minX, query[i].x);
        maxX = std::max(maxX, query[i].x);
        minY = std::min(minY, query[i].y);
        maxY = std::max(maxY, query[i].y);
    }
    if (std::hypot(maxX - minX, maxY - minY) < kMinInlierSpanMm / kMmPerPx) {
        return false;
    }
    return true;
}

// Robot field pose from the patch->map transform and the camera's patch pixel.
// Pushing the camera's own patch pixel through the transform lands on the map
// point under the camera, which is the robot's ground position.
CameraPosition poseFromTransform(const cv::Mat& transform,
                                 const cv::Point2d& cameraPatchPx,
                                 const cv::Point2d& mapOriginPx,
                                 double mapMmPerPx) {
    const double mx = transform.at<double>(0, 0) * cameraPatchPx.x +
                      transform.at<double>(0, 1) * cameraPatchPx.y +
                      transform.at<double>(0, 2);
    const double my = transform.at<double>(1, 0) * cameraPatchPx.x +
                      transform.at<double>(1, 1) * cameraPatchPx.y +
                      transform.at<double>(1, 2);
    const cv::Point2d mm = mapPxToFieldMm(cv::Point2d(mx, my), mapOriginPx, mapMmPerPx);

    CameraPosition position;
    position.x = mm.x;
    position.y = mm.y;
    position.z = CAMERA_HEIGHT_MM;
    // Both images use the image convention (x right, y down), so the fitted
    // angle is clockwise on screen and the field's yaw is anticlockwise; the
    // quarter turn reconciles the patch's "far ground" axis with the heading.
    position.heading = normalizeAngle(
        90.0 - std::atan2(transform.at<double>(1, 0),
                          transform.at<double>(0, 0)) * kRadToDeg);
    return position;
}

// Field-millimetre box of everything the camera could be seeing, from the
// prior. Projects the patch's valid footprint into the robot frame, places it
// at the prior pose through the yaw slack and grows it by the position slack.
// Returns false when the patch has no ground, so the caller matches the whole
// map instead.
bool cameraViewWindow(const cv::Mat& rectified, const GroundWarper& warper,
                      const CameraPosition& prior, double positionSlackMm,
                      double yawSlackDeg, cv::Rect2d& window) {
    cv::Mat content;
    cv::compare(rectified, 0, content, cv::CMP_NE);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(content, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> points;
    for (const std::vector<cv::Point>& contour : contours) {
        points.insert(points.end(), contour.begin(), contour.end());
    }
    if (points.empty()) {
        return false;
    }

    std::vector<cv::Point> hull;
    cv::convexHull(points, hull);

    std::vector<cv::Point2d> robot;
    robot.reserve(hull.size());
    for (const cv::Point& p : hull) {
        robot.push_back(warper.robotMm(cv::Point2d(p.x, p.y)));
    }

    std::vector<double> angles;
    if (yawSlackDeg <= 0.0) {
        angles.push_back(0.0);
    } else {
        for (int i = 0; i < 9; ++i) {
            angles.push_back(-yawSlackDeg + 2.0 * yawSlackDeg * i / 8.0);
        }
    }

    double minX = 1e18, minY = 1e18, maxX = -1e18, maxY = -1e18;
    for (double a : angles) {
        const double r = (prior.heading + a) * kDegToRad;
        const double c = std::cos(r);
        const double s = std::sin(r);
        for (const cv::Point2d& p : robot) {
            const double fx = prior.x + p.x * c - p.y * s;
            const double fy = prior.y + p.x * s + p.y * c;
            minX = std::min(minX, fx);
            maxX = std::max(maxX, fx);
            minY = std::min(minY, fy);
            maxY = std::max(maxY, fy);
        }
    }

    const double pad = std::max(0.0, positionSlackMm);
    window = cv::Rect2d(minX - pad, minY - pad,
                        (maxX - minX) + 2.0 * pad, (maxY - minY) + 2.0 * pad);
    return true;
}

} // namespace

cv::Mat GroundWarper::warp(const cv::Mat& gray) const {
    cv::Mat out;
    cv::warpPerspective(gray, out, homography, out_size, cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT, cv::Scalar(0));
    return out;
}

cv::Point2d GroundWarper::robotMm(const cv::Point2d& px) const {
    const double rawX = px.x * mm_per_px + x0;
    const double rawY = px.y * mm_per_px + y0;
    return cv::Point2d(-rawY, -rawX);
}

cv::Point2d GroundWarper::cameraPatchPx() const {
    return cv::Point2d(-x0 / mm_per_px, -y0 / mm_per_px);
}

FeaturesLocalizer::FeaturesLocalizer()
    : detector_(cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, kAkazeDescriptorSize,
                                  kAkazeDescriptorChannels, kAkazeThreshold)),
      matcher_(cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING, false)) {}

bool FeaturesLocalizer::loadCalibration(const std::string& cameraFilePath) {
    cv::FileStorage fs(cameraFilePath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LOG_ERROR("FeaturesLocalizer - failed to open calibration file ", cameraFilePath);
        return false;
    }
    fs["camera_matrix"] >> camera_matrix_;
    fs.release();

    if (camera_matrix_.rows != 3 || camera_matrix_.cols != 3) {
        LOG_ERROR("FeaturesLocalizer - calibration file ", cameraFilePath,
                  " has no usable camera_matrix");
        camera_matrix_.release();
        calibrated_ = false;
        return false;
    }
    camera_matrix_.convertTo(camera_matrix_, CV_64F);
    calibrated_ = true;
    warper_size_ = cv::Size();
    LOG_INFO("FeaturesLocalizer - loaded calibration from ", cameraFilePath);
    return true;
}

bool FeaturesLocalizer::loadMap(const std::string& mapFilePath) {
    cv::Mat gray = cv::imread(mapFilePath, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
        LOG_ERROR("FeaturesLocalizer - failed to read map ", mapFilePath);
        return false;
    }

    // The file is 1 mm/px; the working pair is kMmPerPx, so resampling here is
    // what lets the map and the patch share a grid and a match stay rigid.
    const double nativeMmPerPx = FIELD_WIDTH_MM / gray.cols;
    if (std::fabs(nativeMmPerPx - kMmPerPx) > 1e-6) {
        const double factor = nativeMmPerPx / kMmPerPx;
        cv::resize(gray, gray,
                   cv::Size(std::max(1, static_cast<int>(std::lround(gray.cols * factor))),
                            std::max(1, static_cast<int>(std::lround(gray.rows * factor)))),
                   0.0, 0.0, cv::INTER_AREA);
    }

    map_gray_ = gray;
    map_mm_per_px_ = FIELD_WIDTH_MM / static_cast<double>(gray.cols);
    map_origin_px_ = cv::Point2d(gray.cols / 2.0 - 0.5, gray.rows / 2.0 - 0.5);

    detector_->detectAndCompute(gray, cv::noArray(), map_keypoints_, map_descriptors_);
    map_points_px_.clear();
    map_points_px_.reserve(map_keypoints_.size());
    for (const cv::KeyPoint& kp : map_keypoints_) {
        map_points_px_.push_back(kp.pt);
    }

    LOG_INFO("FeaturesLocalizer - map ", mapFilePath, " ", gray.cols, "x", gray.rows,
             " (", map_mm_per_px_, " mm/px), ", map_points_px_.size(), " keypoints");
    return true;
}

bool FeaturesLocalizer::buildWarper(const cv::Size& size) {
    if (size.width <= 0 || size.height <= 0) {
        return false;
    }

    cv::Matx33d cameraMatrix;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            cameraMatrix(r, c) = camera_matrix_.at<double>(r, c);
        }
    }

    const cv::Matx33d H =
        captureHomography(cameraMatrix, CAMERA_PITCH_DEG, CAMERA_HEIGHT_MM);

    double x0, y0, x1, y1;
    groundBounds(H, size.width, size.height, x0, y0, x1, y1);
    const double pad = 0.02 * std::max(x1 - x0, y1 - y0);
    x0 -= pad;
    y0 -= pad;
    x1 += pad;
    y1 += pad;

    const double scale = 1.0 / kMmPerPx;
    warper_.mm_per_px = kMmPerPx;
    warper_.x0 = x0;
    warper_.y0 = y0;
    warper_.out_size = cv::Size(
        std::max(1, static_cast<int>(std::lround((x1 - x0) * scale))),
        std::max(1, static_cast<int>(std::lround((y1 - y0) * scale))));

    const cv::Matx33d S(scale, 0.0, -x0 * scale,
                        0.0, scale, -y0 * scale,
                        0.0, 0.0, 1.0);
    warper_.homography = cv::Mat(S * H).clone();
    warper_size_ = size;

    LOG_INFO("FeaturesLocalizer - warp ", warper_.out_size.width, "x",
             warper_.out_size.height, " at ", kMmPerPx, " mm/px");
    return true;
}

bool FeaturesLocalizer::locate(const cv::Mat& frame, CameraPosition& position) {
    if (!calibrated_ || frame.empty() || map_points_px_.empty()) {
        return false;
    }
    if (warper_size_ != frame.size() && !buildWarper(frame.size())) {
        return false;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = frame;
    }

    const cv::Mat rectified = warper_.warp(gray);

    std::vector<cv::KeyPoint> patchKeypoints;
    cv::Mat patchDescriptors;
    detector_->detectAndCompute(rectified, cv::noArray(), patchKeypoints,
                                patchDescriptors);
    if (patchKeypoints.empty() || patchDescriptors.empty()) {
        return false;
    }

    std::vector<cv::Point2d> patchPoints;
    patchPoints.reserve(patchKeypoints.size());
    for (const cv::KeyPoint& kp : patchKeypoints) {
        patchPoints.push_back(kp.pt);
    }

    // Restrict the map to what the prior says the camera can see. Without a
    // prior (or on a degenerate patch) the whole map is offered.
    std::vector<int> candidates;
    candidates.reserve(map_points_px_.size());
    cv::Rect2d window;
    if (hasPrior() && cameraViewWindow(rectified, warper_, prior(), 0.0,
                                       kPriorYawSlackDeg, window)) {
        for (int i = 0; i < static_cast<int>(map_points_px_.size()); ++i) {
            const cv::Point2d mm = mapPxToFieldMm(map_points_px_[i], map_origin_px_,
                                                  map_mm_per_px_);
            if (mm.x >= window.x && mm.x <= window.x + window.width &&
                mm.y >= window.y && mm.y <= window.y + window.height) {
                candidates.push_back(i);
            }
        }
    } else {
        for (int i = 0; i < static_cast<int>(map_points_px_.size()); ++i) {
            candidates.push_back(i);
        }
    }
    if (static_cast<int>(candidates.size()) < 2) {
        return false;
    }

    cv::Mat candidateDescriptors(candidates.size(), map_descriptors_.cols,
                                 map_descriptors_.type());
    std::vector<cv::Point2d> candidatePoints;
    candidatePoints.reserve(candidates.size());
    for (int k = 0; k < static_cast<int>(candidates.size()); ++k) {
        map_descriptors_.row(candidates[k]).copyTo(candidateDescriptors.row(k));
        candidatePoints.push_back(map_points_px_[candidates[k]]);
    }

    std::vector<int> queryIdx, trainIdx;
    matchFeatures(patchDescriptors, candidateDescriptors, *matcher_, kRatioTest,
                  queryIdx, trainIdx);

    std::vector<cv::Point2d> query, train;
    query.reserve(queryIdx.size());
    train.reserve(trainIdx.size());
    for (size_t i = 0; i < queryIdx.size(); ++i) {
        query.push_back(patchPoints[queryIdx[i]]);
        train.push_back(candidatePoints[trainIdx[i]]);
    }

    cv::Mat transform, inlierMask;
    if (!solveRigid(query, train, transform, inlierMask)) {
        return false;
    }

    position = poseFromTransform(transform, warper_.cameraPatchPx(),
                                 map_origin_px_, map_mm_per_px_);
    LOG_INFO("FeaturesLocalizer - fix (", position.x, ", ", position.y,
             ") mm, heading ", position.heading, " deg, ", query.size(),
             " matches, ", cv::countNonZero(inlierMask), " inliers");
    return true;
}

} // namespace vision
