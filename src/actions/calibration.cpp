#include "actions/calibration.h"
#include "main.hpp"
#include "navigation/driveControl.h"
#include "navigation/navigation.h"
#include "utils/logger.hpp"
#include "actions/strats.hpp"

#include <numeric>
#include <vector>

bool calibrate_otos() {
    // Function called continuously from the main loop to calibrate the otosensors
    // Returns true when calibration is done
    static int step = 0;
    static bool state = false; // Used to alternate between two positions

    // Aruco tags are at pos (+/-400, +/-900)
    nav_return_t ret;

    switch (step){
    case 0:{
        // Angle compensation test
        // New Scalar = Current Scalar * (Actual Angle / Reported Angle)
        static bool has_prev_measure = false;
        static bool skip_first_scalar = true;
        static position_t prev_pos = {0.0, 0.0, 0.0};
        static std::vector<double> scalar_angle_samples;

        position_t pos1 = {0.0, 700.0, 90.0};

        if (state)
            pos1.a += 70.0;
        else
            pos1.a -= 70.0;
        if (colorTeam == YELLOW)
            position_robot_flip(pos1);

        ret = navigationGoTo(pos1, false, false, true);

        if (ret == NAV_DONE){
            if (!has_prev_measure){
                prev_pos = nav_prev_final_pos_cam;
                has_prev_measure = true;
                LOG_GREEN_INFO("OTOS angle calibration: first pose recorded");
            } else {
                const double da_cam = normalize_angle(nav_prev_final_pos_cam.a - prev_pos.a);
                const double da_otos = normalize_angle(nav_prev_final_pos_otos.a - prev_pos.a);

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

                prev_pos = nav_prev_final_pos_cam;
            }

            state = !state;

            if (scalar_angle_samples.size() >= 8){
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
                    // drive.setAngularScalar(mean);
                    step = 1;
                    scalar_angle_samples.clear();
                    skip_first_scalar = true;
                    LOG_GREEN_INFO("OTOS calibration done, moving to step 2");
                }
            }
        }
    } break;
    case 1:{
        // Position compensation test, also compensate for otos angle
        // New Scalar = Current Scalar * (Actual Distance / Reported Distance)
        static bool has_prev_measure = false;
        static bool skip_first_scalar = true;
        static position_t prev_nav_prev_final_pos_cam = {0.0, 0.0, 0.0};
        static position_t prev_nav_prev_final_pos_otos = {0.0, 0.0, 0.0};
        static std::vector<double> scalar_dist_samples;

        position_t pos1 = {300.0, 500.0, 90.0};
        if (state)
            pos1.x *= -1.0;

        if (colorTeam == YELLOW)
            position_robot_flip(pos1);
        ret = navigationGoTo(pos1, false, false, true);

        if (ret == NAV_DONE){
            if (!has_prev_measure){
                prev_nav_prev_final_pos_cam = nav_prev_final_pos_cam;
                prev_nav_prev_final_pos_otos = nav_prev_final_pos_otos;
                has_prev_measure = true;
                LOG_GREEN_INFO("OTOS scalar calibration: first pose recorded");
            } else {
                const double d_cam_x = nav_prev_final_pos_cam.x - prev_nav_prev_final_pos_cam.x;
                const double d_cam_y = nav_prev_final_pos_cam.y - prev_nav_prev_final_pos_cam.y;
                const double d_otos_x = nav_prev_final_pos_otos.x - prev_nav_prev_final_pos_otos.x;
                const double d_otos_y = nav_prev_final_pos_otos.y - prev_nav_prev_final_pos_otos.y;

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
                        if (scalar_dist_samples.size() >= 8){
                            scalar_dist_samples.erase(scalar_dist_samples.begin());
                        }
                        scalar_dist_samples.push_back(scalar_dist);
                    }
                } else {
                    LOG_WARNING("OTOS scalar calibration skipped: reported displacement too small");
                }

                prev_nav_prev_final_pos_cam = nav_prev_final_pos_cam;
                prev_nav_prev_final_pos_otos = nav_prev_final_pos_otos;
            }

            state = !state;

            if (scalar_dist_samples.size() >= 8){
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
                    // drive.setLinearScalar(mean);
                    LOG_GREEN_INFO("OTOS calibration done");
                    step ++;
                    scalar_dist_samples.clear();
                    skip_first_scalar = true;
                }
            }
        }
    } break;
    case 2:{
        // Calibrate using a code close to init pos
        position_t pos = StratStartingPos();
        pos.x = -400;
        ret = navigationGoTo(pos, false, true, true);
        if (ret == NAV_RETURN_DONE){
            step++;
            LOG_GREEN_INFO("OTOS calibration done, moving to step 3");
        }
    }break;
    case 3:{
        // goto home
        position_t pos = StratStartingPos();
        ret = navigationGoTo(pos, false, true, true);
        if (ret == NAV_RETURN_DONE){
            LOG_GREEN_INFO("OTOS calibration done, returned to home position");
            step = 0; // Reset for next time            
            return true;
        }
    }break;
    }

    return false; // Not done
}
