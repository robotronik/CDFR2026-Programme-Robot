#pragma once
#include "utils/json.hpp" // For handling JSON
#include <string>
#include "vision/ransac.hpp"

using json = nlohmann::json;

class ArucoCam {   
private:
    int pid;
    int id;
    bool status; // true if the camera is running, false otherwise
public:
    std::vector<block_t> alignBlocks;
    ArucoCam(int cam_number, const char* calibration_file_path);
    ~ArucoCam();

    void start();
    void stop();

    bool getPos(double & x, double & y, double & a, bool& success);
    bool getRobotPos(double & x, double & y, double & a, bool& success);
    bool getObjectData(json& objects, int& sucess);

    bool ToObjectPos(json& data, block_t& start, double& angle, double& lenght, int& success, bool steal);
    bool ToObjectColor(bool* order, int& success);
    bool ToIsolatedObject(json& data, double & x, double & y, double & a, bool& success);

    bool getObjectPos(double & x, double & y, double & a, int& success, bool steal = false);
    bool getObjectInfoColors(bool* order, block_t& start, double& angle, double& lenght, int& success, bool steal = false);
    bool getBestIsolatedObject(double & x, double & y, double & a, bool& success);

    json getBestIsolatedObject_json();
    json getObjectPosition_json();
    json getRobotPosition_json();

private:
    std::string url;
    void reset_tracking();
};