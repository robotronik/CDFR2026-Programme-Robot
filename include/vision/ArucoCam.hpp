#pragma once

#include <string>

class ArucoCam {   
private:
    int pid;
    int id;
    bool status; // true if the camera is running, false otherwise
public:
    ArucoCam(int cam_number, const char* calibration_file_path);
    ~ArucoCam();
    bool getPos(int & x, int & y, int & a);
private:
    std::string url;
    void start();
    void stop();
};