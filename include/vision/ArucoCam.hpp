#pragma once
#include "utils/json.hpp" // For handling JSON
#include <string>
using json = nlohmann::json;

#define OFFSET_CAM_X 129 // Offset of the camera in mm on the x axis
#define OFFSET_CAM_Y 0 // Offset of the camera in mm on the y axis
#define OFFSET_CAM_A 0 // Offset angle of the camera in degrees

class ArucoCam {   
private:
    int pid;
    int id;
    bool status; // true if the camera is running, false otherwise
public:
    ArucoCam(int cam_number, const char* calibration_file_path);
    ~ArucoCam();
    bool getPos(double & x, double & y, double & a, bool& success);
    bool getRobotPos(double & x, double & y, double & a, bool& success);
    bool getObjectData(json& objects, bool& sucess);

    bool ToObjectPos(json& data, double & x, double & y, double & a, bool& success);
    bool ToObjectColor(json& data, uint8_t& order, bool& success);

    bool getObjectColor(uint8_t& order, bool& success);
    bool getObjectPos(double & x, double & y, double & a, bool& success);
private:
    std::string url;
    void start();
    void stop();
};