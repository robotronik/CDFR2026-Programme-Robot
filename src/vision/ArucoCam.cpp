#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <math.h>
#include "utils/httplib.h"
#include "vision/ArucoCam.hpp"
#include "utils/logger.hpp"
#include <algorithm>

#define PORT_OFFSET 5000
#define SCAN_FAIL_FRAMES_NUM 10
#define SCAN_DONE_FRAMES_NUM 20

pid_t startPythonProgram(char** args);
void stopPythonProgram(pid_t pid);
bool restAPI_GET(const std::string &url, const std::string &resquest, json &response);

ArucoCam::ArucoCam(int cam_number, const char* calibration_file_path) {
    id = cam_number;
    if (id < 0) {
        pid=-1;
        status = true;
        LOG_INFO("Emulating ArucoCam");
        return;
    }
    // Child process: Split the args string into individual arguments
    std::string port = std::to_string(PORT_OFFSET + id);

    char* args[] = {
        (char*)"/usr/bin/python3",
        (char*)"pi_detect_aruco.py",
        (char*)calibration_file_path,
        (char*)"--api-port",
        (char*)port.c_str(),
        (char*)"--headless",
        (char*)"True",
        nullptr
    };

    pid = startPythonProgram(args);
    if (pid == -1) {
        LOG_ERROR("Failed to start ArucoCam ", id);
    }
    else
    {
        LOG_GREEN_INFO("ArucoCam ", id, " started with PID ", pid);
        url = "http://localhost:" + std::to_string(PORT_OFFSET + id);
        status = false;
    }
}
ArucoCam::~ArucoCam(){
    if (pid > 0)
        stopPythonProgram(pid);
}

// Returns true when done
bool ArucoCam::getPos(double & x, double & y, double & a, bool& success) {
    success = false;
    if (status == false) {
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " is not running, will start it now");
        start();
        return false;
    }

    // LOG_DEBUG("Fetching position from ArucoCam ", id);
    // Calls /position rest api endpoint of the ArucoCam API
    // Returns true if the call was successful, false otherwise
    if (id < 0) {
        // TODO change this to return a random position
        return true;
    }
    json response;
    if (restAPI_GET(url, "/position", response) == false) {
        LOG_ERROR("ArucoCam::getPos() - Failed to fetch position");
        return true;
    }
    int failedFrames = response.value("failedFrames", -1);
    int sucessFrames = response.value("sucessFrames", -1);
    // int totalFrames = response.value("totalFrames", -1);
    if (failedFrames == -1 || sucessFrames == -1) {
        LOG_ERROR("ArucoCam::getPos() - Invalid response data, camera might not be running");
        status = false;
        return true;
    }
    if (failedFrames > SCAN_FAIL_FRAMES_NUM) {
        LOG_EXTENDED_DEBUG("Cam has too many failed frames : ", failedFrames);
        stop();
        return true;
    }
    if (sucessFrames < SCAN_DONE_FRAMES_NUM) {
        //LOG_EXTENDED_DEBUG("Cam has not enough good success frames : ", sucessFrames);
        return false;
    }
    // Extract the values from the JSON object
    if(response.is_null()){
        success = false;
        return true;
    }
    json position = response["position"];
    if(!position["x"].is_null() && !position["y"].is_null() && !position["a"].is_null()){
        x = position.value("x", 0);
        y = position.value("y", 0);
        a = position.value("a", 0);
    }else{
        success = false;
        return true;
    }
    success = true;
    //LOG_GREEN_INFO("ArucoCam ", id, " position: { x = ", x, ", y = ", y, ", a = ", a, " }");
    // Return true if the values were successfully extracted
    stop();
    return true;
}

bool ArucoCam::getObjectData(json& objects, bool& sucess){
    sucess = false;
    if (status == false) {
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " is not running, will start it now");
        start();
        return false;
    }

    // LOG_DEBUG("Fetching position from ArucoCam ", id);
    // Calls /position rest api endpoint of the ArucoCam API
    // Returns true if the call was successful, false otherwise
    if (id < 0) {
        // TODO change this to return a random position
        return true;
    }
    json response;
    if (restAPI_GET(url, "/objects", response) == false) {
        LOG_ERROR("ArucoCam::getObjectData() - Failed to fetch objects");
        return true;
    }
    // Extract the values from the JSON object
    if(response.is_null()){
        return true;
    }
    sucess = true;
    objects = response;
    return true;
}


typedef struct {
    double x;
    double y;
    double a;
    bool color; //true for Blue false for Yellow
}block_t;

bool sortBlockT(const block_t& a, const block_t& b){ return a.y < b.y;}

//return the order of the color in the stock in binary form
// 0110 meaning Yellow Blue Blue Yellow
bool ArucoCam::getObjectColor(bool* order, bool& success){
    json data;
    bool data_success;
    success = false;
    if(getObjectData(data, data_success)){
        if(!data_success){
            success = false;
            stop();
            return true;
        }
    }else{
        return false;
    }
    return ToObjectColor(data, order, success);

}

bool ArucoCam::ToObjectColor(json& data, bool* order, bool& success){
    int count = 0;
    auto& objects = data["objects"];
    std::vector<block_t> possible;
    for (auto& [key, list] : objects.items()) {
        for (auto& obj : list) {
            
            // On convertit vers le repère robot 
            double a_tag_rad = obj.value("a",0.0) * M_PI / 180.0;
            double sin_tag = sin(a_tag_rad);
            double cos_tag = cos(a_tag_rad);
            double x_tmp = obj.value("x", 0.0);
            double y_tmp = obj.value("y", 0.0);
            //m_x += x_tmp* cos_tag - y_tmp * sin_tag;
            possible.push_back(block_t{
                .y= x_tmp* sin_tag + y_tmp*cos_tag, 
                .color = (obj.value("label", "") == "Blue")? true : false
            }); 
            count++;
        }
    }

    if(possible.empty()){
        success = false;
        stop();
        return true;
    }

    std::sort(possible.begin(), possible.end(), sortBlockT);
    for(size_t i = 0 ; i< (size_t)4; i++){  
        order[i] = possible[i].color;
        if(possible[i].color){
            LOG_DEBUG("Blue");
        }else{
            LOG_DEBUG("Yellow");
        }
    }
    success = true;
    stop();
    return true;
}

// Returns the position of the average position of the aruco tags detected on the table
// Returns true when done
bool ArucoCam::ToObjectPos(json& data, double & x, double & y, double & a, bool& success) {
    
    int count = 0;
    auto& objects = data["objects"];
    std::vector<block_t> visibleBlocks;

    for (auto& [key, list] : objects.items()) {
        for (auto& obj : list) {
            
            // On convertit vers le repère robot 
            double a_tag_rad = -1 * obj.value("a",0.0) * M_PI / 180.0;
            double sin_tag = sin(a_tag_rad);
            double cos_tag = cos(a_tag_rad);
            double x_tmp = obj.value("x", 0.0);
            double y_tmp = obj.value("y", 0.0);
            LOG_EXTENDED_DEBUG("Coord du tag dans le repère du robot ( ", -1*(x_tmp* cos_tag - y_tmp * sin_tag),", ",-1*(x_tmp* sin_tag + y_tmp*cos_tag), ")");
            visibleBlocks.push_back(block_t{
                .x = -1*(x_tmp* cos_tag - y_tmp * sin_tag),
                .y = -1*(x_tmp* sin_tag + y_tmp*cos_tag),
                .a = -1 * obj.value("a",0.0),
                .color = (obj.value("label", "") == "Blue")? true : false
            });
            count++;
        }
    }

    if(!count){
        success = false;
        stop();
        return true;
    }else if (count == 4){
        double m_x = 0, m_y = 0;

        for(size_t i = 0 ; i< (size_t)4; i++){  
            m_x += visibleBlocks[i].x;
            m_y += visibleBlocks[i].y;
        }   
        m_x = m_x / count;
        m_y = m_y / count;
        
        // Décalage pour le centre du robot
        m_x += OFFSET_CAM_X;
        m_y += OFFSET_CAM_Y;

        //projection dans le repère de la table:
        double a_rad = (a) * M_PI / 180.0;
        double cos_a = cos(a_rad);
        double sin_a = sin(a_rad);
        x += m_x * cos_a - m_y * sin_a;
        y += m_x * sin_a + m_y * cos_a;
        success = true;
        LOG_GREEN_INFO("Tag detection ", id, " position: { x = ", x, ", y = ", y, ", a = ", a, " }");
        // Return true if the values were successfully extracted
        stop();
        return true;
    }else{
        LOG_ERROR("Situation with more or less than 4 blocks not yet implemented");
        //TODO
        double m_x = 0, m_y = 0;

        for(size_t i = 0 ; i< (size_t)count; i++){  
            m_x += visibleBlocks[i].x;
            m_y += visibleBlocks[i].y;
        }   
        m_x = m_x / count;
        m_y = m_y / count;
        
        // Décalage pour le centre du robot
        m_x += OFFSET_CAM_X;
        m_y += OFFSET_CAM_Y;

        //projection dans le repère de la table:
        double a_rad = (a) * M_PI / 180.0;
        double cos_a = cos(a_rad);
        double sin_a = sin(a_rad);
        x += m_x * cos_a - m_y * sin_a;
        y += m_x * sin_a + m_y * cos_a;
        success = true;
        LOG_GREEN_INFO("Tag detection ", id, " position: { x = ", x, ", y = ", y, ", a = ", a, " }");
        // Return true if the values were successfully extracted
        stop();
        return true;
    }
}

/*
    Get the most isolated object to take
*/
bool ArucoCam::ToIsolatedObject(json& data, double & x, double & y, double & a, bool& success){
    int count = 0;
    auto& objects = data["objects"];
    std::vector<block_t> possible;
    for (auto& [key, list] : objects.items()) {
        for (auto& obj : list) {
            
            // On convertit vers le repère robot 
            double a_tag_rad = obj.value("a",0.0) * M_PI / 180.0;
            double sin_tag = sin(a_tag_rad);
            double cos_tag = cos(a_tag_rad);
            double x_tmp = obj.value("x", 0.0);
            double y_tmp = obj.value("y", 0.0);
            possible.push_back(block_t{
                .x = x_tmp* cos_tag - y_tmp * sin_tag,
                .y= x_tmp* sin_tag + y_tmp*cos_tag,
                .a = obj.value("a",0.0),
                .color = (obj.value("label", "") == "Blue")? true : false
            }); 
            count++;
        }
    }

    if(possible.empty()){
        success = false;
        return true;
    }

    std::sort(possible.begin(), possible.end(), sortBlockT);
    LOG_GREEN_INFO("Isolated on one side is ", possible[0].color, " at ( ",possible[0].x,", ",possible[0].y,")");
    LOG_GREEN_INFO("Isolated on the other side is ", possible[3].color, " at ( ",possible[3].x,", ",possible[3].y,")");
    x = possible[0].x;
    y = possible[0].y;
    a = possible[0].a;

    success = true;
    return true;
}

bool ArucoCam::getBestIsolatedObject(double & x, double & y, double & a, bool& success){
    json data;
    bool data_success;
    success = false;
    if(getObjectData(data, data_success)){
        if(!data_success){
            success = false;
            stop();
            return true;
        }
    }else{
        return false;
    }

    return ToIsolatedObject(data, x, y, a, success);
}

json ArucoCam::getBestIsolatedObject_json(){
    double x = 0;
    double y = 0;
    double a = 0;
    bool sucess = false;
    while(!getBestIsolatedObject(x,y,a,sucess)){
        continue;
    }
    return json{
        {"x", x},
        {"y", y},
        {"a", a}};
}

bool ArucoCam::getObjectPos(double & x, double & y, double & a, bool& success){
    json data;
    bool data_success;
    success = false;
    if(getObjectData(data, data_success)){
        if(!data_success){
            success = false;
            return true;
        }
    }else{
        return false;
    }

    return ToObjectPos(data, x, y, a, success);
}

json ArucoCam::getObjectPosition_json(){
    double x = 0;
    double y = 0;
    double a = 0;
    bool sucess = false;
    while(!getObjectPos(x,y,a,sucess)){
        continue;
    }
    return json{
        {"x", x},
        {"y", y},
        {"a", a}};
}

bool ArucoCam::getObjectInfoColors(bool* order, double & x, double & y, double & a, bool& success){
    json data;
    bool data_success;
    success = false;
    if(getObjectData(data, data_success)){
        if(!data_success){
            success = false;
            return true;
        }
    }else{
        return false;
    }

    if(ToObjectPos(data, x, y, a, success)){// Can not return False so first If is useless
        if(success){
            if(ToObjectColor(data,order, success)){ // Can not return false either
                return true; // exec finished 
            }else return false; // exec unfinished
        }else return true; // exec finished on fail
    }else return false; // exec unfinished

}

void ArucoCam::start() {
    json response;
    if (restAPI_GET(url, "/start", response)){
        status = true;
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " started");
    }
}

void ArucoCam::stop() {
    json response;
    if (restAPI_GET(url, "/stop", response)){
        status = false;
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " stopped");
    }
}

bool ArucoCam::getRobotPos(double & x, double & y, double & a, bool& success) {
    // Camera offset from robot center in mm and degrees
    bool result = getPos(x, y, a, success);
    if (result && success) {
        // Convert camera position to robot position
        double cam_a_rad = a * M_PI / 180.0;
        double cos_a = cos(cam_a_rad);
        double sin_a = sin(cam_a_rad);
        x -= OFFSET_CAM_X * cos_a - OFFSET_CAM_Y * sin_a;
        y -= OFFSET_CAM_X * sin_a + OFFSET_CAM_Y * cos_a;
        a += OFFSET_CAM_A;
        // Normalize angle to ]-180;180]
        if (a > 180.0) a -= 360.0;
        else if (a <= -180.0) a += 360.0;
    }
    return result;    
}

bool restAPI_GET(const std::string &url, const std::string &resquest, json &response) {
    // HTTP
    httplib::Client cli(url);
    auto res = cli.Get(resquest.c_str());
    // Check for nullptr
    if (!res) {
        LOG_ERROR("Failed to fetch response from ", url, resquest);
        return false;
    }
    // LOG_GREEN_INFO("HTML Status is ", res->status);
    // LOG_GREEN_INFO("HTML Body is ", res->body);

    // Check if the response code is 200 (OK)
    if (res->status != 200) {
        LOG_ERROR("HTTP error: ", res->status);
        return false;
    }
    try {
        // Parse JSON response
        response = json::parse(res->body);
        // LOG_GREEN_INFO("API Response: ", response.dump(4));
        return true;
    } catch (const json::parse_error& e) {
        LOG_ERROR("JSON parse error: ", e.what());
        return false;
    }
}

pid_t startPythonProgram(char ** args) {
    pid_t pid = fork();

    setenv("HOME", "/home/robotronik", 1);
    setenv("USER", "robotronik", 1);
    setenv("PYTHONPATH", "/home/robotronik/.local/lib/python3.13/site-packages", 1);

    if (pid == -1) {
        LOG_ERROR("startPythonProgram - Failed to fork process");
        return -1;
    } else if (pid == 0) {
        // Execute the Python program with arguments
        execvp(args[0], args);
        LOG_ERROR("startPythonProgram - Failed to execute Python script");
        _exit(1); // Ensure child process exits
    }

    // Parent process: Return child PID
    return pid;
}

void stopPythonProgram(pid_t pid) {
    if (pid <= 0) {
        LOG_ERROR("stopPythonProgram - Invalid PID");
        return;
    }

    if (kill(pid, SIGTERM) == -1) {
        LOG_ERROR("stopPythonProgram - Failed to terminate process with PID ", pid);
    } else {
        LOG_INFO("stopPythonProgram - Process with PID ", pid, " terminated");
    }
}

size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
    size_t totalSize = size * nmemb;
    if (response) {
        response->append(static_cast<char*>(contents), totalSize);
    }
    return totalSize;
}