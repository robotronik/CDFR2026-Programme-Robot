#include "actions/calibration.h"
#include "actions/RobotControl.hpp"
#include "navigation/navigation.h"
#include "vision/Cam.hpp"
#include "vision/ArucoLocalizer.hpp"
#include "utils/logger.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

bool calibrate_otos(TableState* tableStatus, RobotControl* robot, Cam* cam, position_t startPos, bool skip_calibration) {
    // Function called continuously from the main loop to calibrate the otosensors
    // Returns true when calibration is done

    static int step = 0; // Entry step: it picks where this run starts, so skip_calibration is always honoured
    static bool state = false; // Used to alternate between two positions

    // Aruco tags are at pos (+/-400, +/-900)
    NavReturn ret;

    switch (step){
    case 0:{
        // Pick the entry point of this calibration run (1: full, 4: straight home)
        step = skip_calibration ? 4 : 1;
    } break;
    case 1:{
        // Angle compensation test
        // New Scalar = Current Scalar * (Actual Angle / Reported Angle)
        static bool has_prev_measure = false;
        static bool skip_first_scalar = true;
        static position_t prev_pos = {0.0, 0.0, 0.0};
        static position_t prev_pos_otos = {0.0, 0.0, 0.0};
        static std::vector<double> scalar_angle_samples;

        position_t pos1 = {0.0, 1100.0, -90.0};

        if (state)
            pos1.a += 70.0;
        else
            pos1.a -= 70.0;
        if (tableStatus->colorTeam == ColorTeam::Yellow)
            position_robot_flip(pos1);

        // Raw OTOS pose, captured before goTo() applies the camera fix at the end of the move
        const position_t otos_pos = robot->drive.position;
        ret = navigation.goTo(pos1, false, false, true);

        if (ret == NavReturn::Done){
            position_t camera_pos;
            if (!cam->getLocalisation(camera_pos)){
                LOG_WARNING("OTOS angle calibration skipped: no camera localisation");
                return false;
            }
            const position_t cam_pos = cameraToRobot(camera_pos);

            if (!has_prev_measure){
                prev_pos = cam_pos;
                prev_pos_otos = otos_pos;
                has_prev_measure = true;
                LOG_GREEN_INFO("OTOS angle calibration: first pose recorded");
            } else {
                const double da_cam = normalize_angle(cam_pos.a - prev_pos.a);
                const double da_otos = normalize_angle(otos_pos.a - prev_pos_otos.a);

                if (fabs(da_otos) > 1e-3){
                    const double scalar_angle = da_cam / da_otos;
                    LOG_GREEN_INFO("OTOS scalar angle=", scalar_angle,
                                   " (actual=", da_cam, " reported=", da_otos, ")");

                    if (skip_first_scalar){
                        skip_first_scalar = false;
                        LOG_GREEN_INFO("OTOS angle calibration: first scalar ignored");
                    } else {
                        if (scalar_angle_samples.size() >= 8){
                            scalar_angle_samples.erase(scalar_angle_samples.begin());
                        }
                        scalar_angle_samples.push_back(scalar_angle);
                    }
                } else {
                    LOG_WARNING("OTOS angle calibration skipped: reported angle too small");
                }

                prev_pos = cam_pos;
                prev_pos_otos = otos_pos;
            }

            state = !state;

            if (scalar_angle_samples.size() >= 6){
                const double mean = std::accumulate(scalar_angle_samples.begin(),
                                                    scalar_angle_samples.end(),
                                                    0.0) /
                                    static_cast<double>(scalar_angle_samples.size());
                bool within_variance = mean != 0.0;
                if (within_variance){
                    const double max_delta = fabs(mean) * 0.01;
                    for (double sample : scalar_angle_samples){
                        if (fabs(sample - mean) > max_delta){
                            within_variance = false;
                            break;
                        }
                    }
                }

                if (within_variance){
                    float current_scalar = robot->drive.getAngularScalar();
                    float new_scalar = current_scalar * mean;
                    robot->drive.setAngularScalar(new_scalar);
                    step = 2;
                    scalar_angle_samples.clear();
                    skip_first_scalar = true;
                    LOG_GREEN_INFO("OTOS calibration done with mean = ", new_scalar);
                }
            }
        }
    } break;
    case 2:{
        // Position compensation test, also compensate for otos angle
        // New Scalar = Current Scalar * (Actual Distance / Reported Distance)
        constexpr int kCalibrationRuns = 2;
        static bool has_prev_measure = false;
        static bool skip_first_scalar = true;
        static position_t prev_cam_pos = {0.0, 0.0, 0.0};
        static position_t prev_otos_pos = {0.0, 0.0, 0.0};
        static std::vector<double> scalar_dist_samples;
        static int calibration_runs = 0;

        position_t pos1 = {0.0f, 1250.0f, -90.0f};
        pos1.x += state ? 400.0 : -400.0;

        if (tableStatus->colorTeam == ColorTeam::Yellow)
            position_robot_flip(pos1);

        // Raw OTOS pose, captured before goTo() applies the camera fix at the end of the move
        const position_t otos_pos = robot->drive.position;
        ret = navigation.goTo(pos1, false, false, true);

        if (ret == NavReturn::Done){
            position_t camera_pos;
            if (!cam->getLocalisation(camera_pos)){
                LOG_WARNING("OTOS scalar calibration skipped: no camera localisation");
                return false;
            }
            const position_t cam_pos = cameraToRobot(camera_pos);

            if (!has_prev_measure){
                prev_cam_pos = cam_pos;
                prev_otos_pos = otos_pos;
                has_prev_measure = true;
                LOG_GREEN_INFO("OTOS scalar calibration: first pose recorded");
            } else {
                const double d_cam_x = cam_pos.x - prev_cam_pos.x;
                const double d_cam_y = cam_pos.y - prev_cam_pos.y;
                const double d_otos_x = otos_pos.x - prev_otos_pos.x;
                const double d_otos_y = otos_pos.y - prev_otos_pos.y;

                const double dist_cam = sqrt(d_cam_x * d_cam_x + d_cam_y * d_cam_y);
                const double dist_otos = sqrt(d_otos_x * d_otos_x + d_otos_y * d_otos_y);

                if (dist_otos > 1e-6){
                    const double scalar_dist = dist_cam / dist_otos;
                    LOG_GREEN_INFO("OTOS scalar distance=", scalar_dist,
                                   " (actual=", dist_cam, " reported=", dist_otos, ")");

                    if (skip_first_scalar){
                        skip_first_scalar = false;
                        LOG_GREEN_INFO("OTOS distance calibration: first scalar ignored");
                    } else {
                        if (scalar_dist_samples.size() >= 6){
                            scalar_dist_samples.erase(scalar_dist_samples.begin());
                        }
                        scalar_dist_samples.push_back(scalar_dist);
                    }
                } else {
                    LOG_WARNING("OTOS scalar calibration skipped: reported displacement too small");
                }

                prev_cam_pos = cam_pos;
                prev_otos_pos = otos_pos;
            }

            state = !state;

            if (scalar_dist_samples.size() >= 6){
                const double mean = std::accumulate(scalar_dist_samples.begin(),
                                                    scalar_dist_samples.end(),
                                                    0.0) /
                                    static_cast<double>(scalar_dist_samples.size());
                bool within_variance = mean != 0.0;
                if (within_variance){
                    const double max_delta = fabs(mean) * 0.01;
                    for (double sample : scalar_dist_samples){
                        if (fabs(sample - mean) > max_delta){
                            within_variance = false;
                            break;
                        }
                    }
                }

                if (within_variance){
                    float current_scalar = robot->drive.getLinearScalar();
                    float new_scalar = current_scalar * mean;
                    robot->drive.setLinearScalar(new_scalar); // TODO Bugfix needed, this causes the OTOS to crash
                    LOG_GREEN_INFO("OTOS calibration done with mean = ", new_scalar);
                    scalar_dist_samples.clear();
                    skip_first_scalar = true;
                    calibration_runs++;
                    if (calibration_runs >= kCalibrationRuns){
                        step++;
                        calibration_runs = 0;
                    }
                }
            }
        }
    } break;
    case 3:{
        // Calibrate using a code close to init pos
        position_t pos = {0, 1250, -45.0};
        if (tableStatus->colorTeam == ColorTeam::Yellow)
            position_robot_flip(pos);
            
        ret = navigation.goTo(pos, false, true, true);
        if (ret == NavReturn::Done){
            step++;
            LOG_GREEN_INFO("OTOS calibration done, moving to home position");
        }
    }break;
    case 4:{
        // goto home
        position_t pos = startPos;
        ret = navigation.goTo(pos, false, true, true);
        if (ret == NavReturn::Done){
            LOG_GREEN_INFO("OTOS calibration done, returned to home position");
            step = 0; // Reset for next time: the entry step re-applies skip_calibration
            return true;
        }
    }break;
    }

    return false; // Not done
}

namespace {

// Optical frame (x right, y down, z forward) to a body frame (x forward,
// y left, z up), so the Euler angles read as the physical camera mounting.
const cv::Matx33d kOpticalToBody(
     0.0, -1.0,  0.0,
     0.0,  0.0, -1.0,
     1.0,  0.0,  0.0);

// Landmark tag frame to the table frame, matching ArucoLocalizer: the tags lie
// flat on the table with their axes swapped onto the field axes.
const cv::Matx33d kTagToTable(
     0.0, -1.0, 0.0,
     1.0,  0.0, 0.0,
     0.0,  0.0, 1.0);

// Number of headings swept between the two colour-side tags.
constexpr int kSweepSamples = 6;

cv::Matx33d rotationZ(double angleDeg) {
    const double a = angleDeg * DEG_TO_RAD;
    const double c = std::cos(a);
    const double s = std::sin(a);
    return cv::Matx33d(c, -s, 0.0,
                       s,  c, 0.0,
                       0.0, 0.0, 1.0);
}

// Chordal L2 mean of rotations, projected back onto SO(3) via SVD.
cv::Matx33d meanRotation(const std::vector<cv::Matx33d>& rotations) {
    cv::Mat sum = cv::Mat::zeros(3, 3, CV_64F);
    for (const cv::Matx33d& r : rotations) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                sum.at<double>(row, col) += r(row, col);
            }
        }
    }

    cv::Mat w, u, vt;
    cv::SVD::compute(sum, w, u, vt);
    cv::Mat mean = u * vt;
    if (cv::determinant(mean) < 0.0) {
        cv::Mat fix = cv::Mat::eye(3, 3, CV_64F);
        fix.at<double>(2, 2) = -1.0;
        mean = u * fix * vt;
    }

    cv::Matx33d result;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            result(row, col) = mean.at<double>(row, col);
        }
    }
    return result;
}

// ZYX Euler angles (degrees) of a body -> reference rotation.
void eulerZYX(const cv::Matx33d& r, double& roll, double& pitch, double& yaw) {
    yaw = std::atan2(r(1, 0), r(0, 0)) * RAD_TO_DEG;
    pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0)) * RAD_TO_DEG;
    roll = std::atan2(r(2, 1), r(2, 2)) * RAD_TO_DEG;
}

struct CameraSample {
    position_t robot;       // raw OTOS robot pose at rest
    position_t camera;      // camera pose on the table from the landmark tags
    double cameraHeight = 0.0; // camera height above the table, mm
    bool hasHeight = false;
};

// Computes and logs the camera mount from the collected samples.
void reportCameraCalibration(const std::vector<CameraSample>& samples,
                             const std::vector<cv::Matx33d>& mountRotations) {
    // Yaw offset: camera heading minus robot heading, averaged circularly.
    double sumCos = 0.0;
    double sumSin = 0.0;
    for (const CameraSample& sample : samples) {
        const double dyaw = (sample.camera.a - sample.robot.a) * DEG_TO_RAD;
        sumCos += std::cos(dyaw);
        sumSin += std::sin(dyaw);
    }
    const double offsetA = std::atan2(sumSin, sumCos) * RAD_TO_DEG;

    // x/y offset in the robot frame. Rotating in place keeps the (unknown)
    // robot position fixed, so comparing every sample to the first one:
    //   camera_i - camera_0 = (Rz(robot_a_i) - Rz(robot_a_0)) * offset
    // cancels it out. Accumulated as a least-squares normal equation.
    const CameraSample& reference = samples.front();
    const double refA = reference.robot.a * DEG_TO_RAD;
    double denom = 0.0;
    double numX = 0.0;
    double numY = 0.0;
    for (size_t i = 1; i < samples.size(); ++i) {
        const double a = samples[i].robot.a * DEG_TO_RAD;
        const double p = std::cos(a) - std::cos(refA);
        const double q = std::sin(a) - std::sin(refA);
        const double bx = samples[i].camera.x - reference.camera.x;
        const double by = samples[i].camera.y - reference.camera.y;

        denom += p * p + q * q;
        numX += p * bx + q * by;
        numY += -q * bx + p * by;
    }
    if (denom < 1e-6) {
        LOG_ERROR("Camera calibration failed: the robot did not rotate between samples");
        return;
    }
    const double offsetX = numX / denom;
    const double offsetY = numY / denom;

    LOG_GREEN_INFO("Camera offset in the robot frame: x=", offsetX, " mm, y=", offsetY,
                   " mm, a=", offsetA, " deg");

    double heightSum = 0.0;
    int heightCount = 0;
    for (const CameraSample& sample : samples) {
        if (sample.hasHeight) {
            heightSum += sample.cameraHeight;
            ++heightCount;
        }
    }
    if (heightCount > 0) {
        LOG_GREEN_INFO("Camera height above the table: ", heightSum / heightCount, " mm");
    }

    if (mountRotations.empty()) {
        LOG_WARNING("Camera calibration: no tag pose available for the Euler rotation");
        return;
    }
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    eulerZYX(meanRotation(mountRotations), roll, pitch, yaw);
    LOG_GREEN_INFO("Camera mount Euler rotation (robot frame): roll=", roll, " deg, pitch=",
                   pitch, " deg, yaw=", yaw, " deg");
}

} // namespace

bool calibrate_camera(TableState* tableStatus, RobotControl* robot, Cam* cam) {
    // Manual camera-mount calibration, to run only after the camera is moved.
    // The robot does not translate: it rotates in place so the camera looks at
    // the two landmark tags on its colour side. Comparing the camera pose (from
    // the tags) with the raw OTOS pose over the sweep recovers the camera offset
    // in the robot frame, and the tag poses give the mount's Euler rotation.
    // Stepped by the main FSM: returns true once the sweep is done and logged.

    static int step = 0;
    static position_t start = {0.0, 0.0, 0.0};
    static position_t targets[kSweepSamples];
    static int targetIndex = 0;
    static std::vector<CameraSample> samples;
    static std::vector<cv::Matx33d> mountRotations;

    switch (step) {
    case 0: {
        if (tableStatus->colorTeam == ColorTeam::None) {
            LOG_WARNING("Camera calibration needs a known team colour");
            return true; // Nothing to do; avoid blocking the FSM.
        }

        // The two landmark tags on the team's colour side share the y sign of
        // the team's starting position.
        const double sideSign = (tableStatus->colorTeam == ColorTeam::Yellow) ? -1.0 : 1.0;
        std::vector<cv::Point2d> sideTags;
        for (const auto& entry : vision::ArucoLocalizer::landmarkTags()) {
            if ((entry.second.y > 0.0) == (sideSign > 0.0)) {
                sideTags.push_back(entry.second);
            }
        }
        if (sideTags.size() != 2) {
            LOG_ERROR("Camera calibration found ", sideTags.size(),
                      " colour-side tags, expected 2");
            return true;
        }

        start = robot->drive.position;

        // Robot headings that point the camera at each of the two tags.
        const double headingA = normalize_angle(
            position_angle(start, {sideTags[0].x, sideTags[0].y, 0.0}) * RAD_TO_DEG);
        const double headingB = normalize_angle(
            position_angle(start, {sideTags[1].x, sideTags[1].y, 0.0}) * RAD_TO_DEG);
        const double sweepSpan = normalize_angle(headingB - headingA);

        for (int i = 0; i < kSweepSamples; ++i) {
            const double t = (kSweepSamples == 1)
                                 ? 0.0
                                 : static_cast<double>(i) / static_cast<double>(kSweepSamples - 1);
            targets[i] = {start.x, start.y, normalize_angle(headingA + sweepSpan * t)};
        }

        samples.clear();
        mountRotations.clear();
        targetIndex = 0;
        step = 1;

        LOG_GREEN_INFO("Camera calibration: sweeping ", kSweepSamples, " headings from ", headingA,
                       " to ", headingB, " deg, robot at (", start.x, ", ", start.y, ")");
    } break;
    case 1: {
        // Rotate in place through the drive board directly: navigation would
        // apply a camera fix (with the old offset) at the end of the move and
        // overwrite the OTOS heading we need to measure the yaw offset.
        if (robot->drive.drive(&targets[targetIndex], 1, false, true)) {
            position_t cameraPos;
            if (!cam->getLocalisation(cameraPos)) {
                LOG_WARNING("Camera calibration: no camera localisation at ",
                            targets[targetIndex].a, " deg");
            } else {
                const position_t robotPos = robot->drive.position;
                CameraSample sample;
                sample.robot = robotPos;
                sample.camera = cameraPos;

                // Camera orientation and height in the robot frame, from each
                // visible tag.
                for (const vision::DetectionResult& detection : cam->getDetections()) {
                    if (!detection.hasPose ||
                        vision::ArucoLocalizer::fieldPosition(detection.id) == nullptr) {
                        continue;
                    }
                    cv::Matx33d tagToCamera;
                    cv::Rodrigues(detection.rvec, tagToCamera);
                    const cv::Matx33d cameraToTable = kTagToTable * tagToCamera.t();
                    const cv::Matx33d cameraToRobot = rotationZ(-robotPos.a) * cameraToTable;
                    mountRotations.push_back(cameraToRobot * kOpticalToBody);

                    if (!sample.hasHeight) {
                        // Camera position in the tag frame; z is the height above
                        // the table (same value ArucoLocalizer reports as z).
                        const cv::Vec3d cameraInTag = -(tagToCamera.t() * detection.tvec);
                        sample.cameraHeight = cameraInTag[2];
                        sample.hasHeight = true;
                    }
                }
                samples.push_back(sample);

                LOG_DEBUG("Camera calibration sample at ", targets[targetIndex].a,
                          " deg: robot (", robotPos.x, ", ", robotPos.y, ", ", robotPos.a,
                          "), camera (", cameraPos.x, ", ", cameraPos.y, ", ", cameraPos.a,
                          "), height ", sample.cameraHeight, " mm");
            }

            ++targetIndex;
            if (targetIndex >= kSweepSamples) {
                step = 2;
            }
        }
    } break;
    case 2: {
        // Back to the starting heading, then report.
        if (robot->drive.drive(&start, 1, false, true)) {
            if (samples.size() < 2) {
                LOG_ERROR("Camera calibration failed: only ", samples.size(),
                          " usable sample(s)");
            } else {
                reportCameraCalibration(samples, mountRotations);
            }
            step = 0;
            return true;
        }
    } break;
    }

    return false; // Not done
}
