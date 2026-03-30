#pragma once
#include "utils/json.hpp" // For handling JSON
#include <string>
#include "vision/ransac.hpp"

using json = nlohmann::json;

#define OFFSET_CAM_X 129 // Offset of the camera in mm on the x axis
#define OFFSET_CAM_Y 0 // Offset of the camera in mm on the y axis
#define OFFSET_CAM_A 0 // Offset angle of the camera in degrees
#define OFFSET_CLAW_Y -32 // Offset to align claw with block, diminuer = plus à droite
#define OFFSET_STOCK 300
#define MULT_PARAM 0.68

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

    bool ToObjectPos(json& data, double & x, double & y, double & a, int& success);
    bool ToObjectColor(bool* order, int& success);
    bool ToIsolatedObject(json& data, double & x, double & y, double & a, bool& success);

    bool getObjectPos(double & x, double & y, double & a, int& success);
    bool getObjectInfoColors(bool* order, double & x, double & y, double & a, int& success);
    bool getBestIsolatedObject(double & x, double & y, double & a, bool& success);

    json getBestIsolatedObject_json();
    json getObjectPosition_json();
    json getRobotPosition_json();

private:
    std::string url;
    void reset_tracking();
};