#pragma once
#include "utils/json.hpp" // For handling JSON
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include "vision/ArucoDetector.hpp"
#include "vision/ransac.hpp"

using json = nlohmann::json;

#define OFFSET_CAM_X 129 // Offset of the camera in mm on the x axis
#define OFFSET_CAM_Y 4.5 // Offset of the camera in mm on the y axis
#define OFFSET_CAM_A 0 // Offset angle of the camera in degrees
#define OFFSET_CLAW_Y -26 // Offset to align claw with block, diminuer = plus à droite
#define OFFSET_STOCK 330
#define MULT_PARAM 0.68

class ArucoCam {   
private:
    int id = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;
    vision::ArucoDetector detector_;

    // Shared detection state, protected by stateMutex_.
    struct State {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double a = 0.0;
        bool hasPosition = false;
        int successFrames = 0;
        int failedFrames = 0;
        json objects = json::object();
    };
    State state_;
    mutable std::mutex stateMutex_;
    bool waitingForPos_ = false;

    void workerLoop();
    bool processDetections(const std::vector<vision::DetectionResult>& detections);
    void addAveragePosition(double x, double y, double z, double a);
public:
    std::vector<block_t> alignBlocks;
    ArucoCam(int cam_number, const char* calibration_file_path);
    ~ArucoCam();

    bool start();
    void stop();

    bool getPos(double & x, double & y, double & a, bool& success);
    bool getRobotPos(double & x, double & y, double & a, bool& success);
    bool getObjectData(json& objects, int& sucess);

    bool ToObjectPos(json& data, double & x, double & y, double & a, int& success);
    bool ToObjectSweep(bool* order, json& data, double & x, double & y, double & a, double & dist_balayage, int& success);

    bool ToObjectColor(bool* order, int& success);
    bool ToIsolatedObject(json& data, double & x, double & y, double & a, bool& success);

    bool getObjectPos(double & x, double & y, double & a, int& success);
    bool getObjectInfoColors(bool* order, double & x, double & y, double & a, int& success);
    bool getObjectForSweep(bool* order, double & x, double & y, double & a, int& success, double& dist_balayage);

    bool getBestIsolatedObject(double & x, double & y, double & a, bool& success);

    json getBestIsolatedObject_json();
    json getObjectPosition_json();
    json getRobotPosition_json();

private:
    void reset_tracking();
};