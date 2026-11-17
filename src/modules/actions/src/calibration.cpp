#include "actions/calibration.h"
#include "actions/RobotControl.hpp"
#include "navigation/navigation.h"
#include "vision/Cam.hpp"
#include "utils/logger.hpp"
#include <numeric>
#include <vector>

bool calibrate_otos(TableState* tableStatus, RobotControl* robot, Cam* cam, position_t startPos, bool skip_calibration) {
    // Function called continuously from the main loop to calibrate the otosensors
    // Returns true when calibration is done

    static int step = skip_calibration ? 2 : 1;
    static bool state = false; // Used to alternate between two positions

    // Aruco tags are at pos (+/-400, +/-900)
    NavReturn ret;

    switch (step){
    case 0:{
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
                    step = 1;
                    scalar_angle_samples.clear();
                    skip_first_scalar = true;
                    LOG_GREEN_INFO("OTOS calibration done with mean = ", new_scalar);
                }
            }
        }
    } break;
    case 1:{
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
    case 2:{
        // Calibrate using a code close to init pos
        position_t pos = {-400, 1250, -90.0};
        if (tableStatus->colorTeam == ColorTeam::Yellow)
            position_robot_flip(pos);
            
        ret = navigation.goTo(pos, false, true, true);
        if (ret == NavReturn::Done){
            step++;
            LOG_GREEN_INFO("OTOS calibration done, moving to step 3");
        }
    }break;
    case 3:{
        // goto home
        position_t pos = startPos;
        ret = navigation.goTo(pos, false, true, true);
        if (ret == NavReturn::Done){
            LOG_GREEN_INFO("OTOS calibration done, returned to home position");
            step = 0; // Reset for next time            
            return true;
        }
    }break;
    }

    return false; // Not done
}
