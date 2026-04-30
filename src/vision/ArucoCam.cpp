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
#define SCAN_FAIL_FRAMES_NUM 2
#define SCAN_DONE_FRAMES_NUM 10

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
        url = "http://localhost:" + std::to_string(PORT_OFFSET + id);
        status = false;
        LOG_GREEN_INFO("ArucoCam ", id, " started with PID ", pid, " at URL ", url);
    }
}
ArucoCam::~ArucoCam(){
    if (pid > 0)
        stopPythonProgram(pid);
}

// Returns true when done
bool ArucoCam::getPos(double & x, double & y, double & a, bool& success) {
    static bool waiting_for_pos = false; // Wether reset tracking or not
    success = false;
    if (id < 0) {
        // TODO change this to return a random position
        return true;
    }
    if (status == false) { // Camera is not running
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " is not running, will start it now");
        start();
        waiting_for_pos = true;
        return false;
    }
    if (!waiting_for_pos){ // Reset tracking
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " starting tracking");
        reset_tracking();
        waiting_for_pos = true;
        return false;
    }

    // LOG_DEBUG("Fetching position from ArucoCam ", id);
    // Calls /position rest api endpoint of the ArucoCam API
    // Returns true if the call was successful, false otherwise
    json response;
    if (restAPI_GET(url, "/position", response) == false) {
        LOG_ERROR("ArucoCam::getPos() - Failed to fetch position");
        waiting_for_pos = false;
        return true;
    }

    if (!response.contains("position") || response["position"].is_null() || !response["position"].is_object()) {
        LOG_ERROR("ArucoCam::getPos() - response missing or invalid 'position'");
        waiting_for_pos = false;
        return true;
    }

    int failedFrames = response.value("failedFrames", -1);
    int sucessFrames = response.value("sucessFrames", -1);
    // int totalFrames = response.value("totalFrames", -1);
    if (failedFrames == -1 || sucessFrames == -1) {
        LOG_ERROR("ArucoCam::getPos() - Invalid response data, camera might not be running");
        waiting_for_pos = false;
        return true;
    }
    if (failedFrames > SCAN_FAIL_FRAMES_NUM) {
        LOG_EXTENDED_DEBUG("Cam has too many failed frames : ", failedFrames);
        waiting_for_pos = false;
        return true;
    }
    if (sucessFrames < SCAN_DONE_FRAMES_NUM) {
        //LOG_EXTENDED_DEBUG("Cam has not enough good success frames : ", sucessFrames);
        return false; 
    }

    json position = response["position"];
    x = position.value("x", -1.0);
    y = position.value("y", -1.0);
    a = position.value("a", -1.0);
    success = true;
    // LOG_GREEN_INFO("ArucoCam ", id, " position: { x = ", x, ", y = ", y, ", a = ", a, " } with sucess frames : ", sucessFrames, " and failed frames : ", failedFrames);
    // Return true if the values were successfully extracted
    waiting_for_pos = false;
    return true;
}

bool ArucoCam::getObjectData(json& objects, int& sucess){
    sucess = -1;
    if (id < 0) {
        // TODO change this to return a random position
        return true;
    }
    if (status == false) {
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " is not running, will start it now");
        start();
        return false;
    }

    // LOG_DEBUG("Fetching position from ArucoCam ", id);
    // Calls /position rest api endpoint of the ArucoCam API
    // Returns true if the call was successful, false otherwise
    json response;
    if (restAPI_GET(url, "/objects", response) == false) {
        LOG_ERROR("ArucoCam::getObjectData() - Failed to fetch objects");
        return true;
    }
    // Extract the values from the JSON object
    if(response.is_null()){
        LOG_ERROR("ArucoCam::getObjectData() - response is null, camera might not be running");
        return true;
    }
    sucess = 0;
    objects = response;
    return true;
}

bool sortBlockT(const block_t& a, const block_t& b){ return a.y < b.y;}

bool ArucoCam::ToObjectColor(bool* order, int& success){
    // Edge case management
    if(alignBlocks.empty()){
        success = -2;
        LOG_ERROR("No objects but still searching for colors. Should not execute");
        return true;
    }else if (alignBlocks.size()>4)
    {
        success = -1;
        LOG_ERROR("To many objects found Error. Should not execute");
        return true;
    }
    
    //proper color detection
    size_t start = 0;
    if(alignBlocks.size()<=2){
        start = 1;
    }
    for(size_t i =  0; i< alignBlocks.size() + start; i++){  
        order[start + i ] = alignBlocks[i].color;
        if(alignBlocks[i].color){
            LOG_EXTENDED_DEBUG("Blue");
        }else{
            LOG_EXTENDED_DEBUG("Yellow");
        }
    }
    success = alignBlocks.size();
    return true;
}

// Returns the position of the average position of the aruco tags detected on the table
// Returns true when done
bool ArucoCam::ToObjectPos(json& data, double & x, double & y, double & a, int& success) {

    if (!data.contains("objects") || data["objects"].is_null() || !data["objects"].is_object()) {
        LOG_ERROR("ArucoCam::ToObjectPos() - invalid or missing 'objects'");
        success = -1;
        return true;
    }else{
        success = -2; // l'erreur n'est plus une erreur de caméra
    }

    int count = 0;
    auto& objects = data["objects"];
    std::vector<block_t> visibleBlocks;
    alignBlocks.clear();
    double tmp_x = 0;
    double tmp_y = 0;
    double tmp_a = 0;

    for (auto& [key, list] : objects.items()) {
        for (auto& obj : list) {
            
            // On convertit vers le repère robot 
            double a_tag_rad = -1 * obj.value("a",0.0) * M_PI / 180.0;
            double sin_tag = sin(a_tag_rad);
            double cos_tag = cos(a_tag_rad);
            double x_tmp = obj.value("x", 0.0);
            double y_tmp = obj.value("y", 0.0);
            //LOG_EXTENDED_DEBUG("Coord du tag dans le repère du robot ( ", -1*(x_tmp* cos_tag - y_tmp * sin_tag),", ",-1*(x_tmp* sin_tag + y_tmp*cos_tag),", ", -1 * obj.value("a",0.0), ")");
            visibleBlocks.push_back(block_t{
                .x = -1*(x_tmp* cos_tag - y_tmp * sin_tag),
                .y = -1*(x_tmp* sin_tag + y_tmp*cos_tag),
                .a = -1 * obj.value("a",0.0),
                .color = (obj.value("label", "") == "Blue")? true : false
            });
            count++;
        }
    }
    LOG_EXTENDED_DEBUG("Found ", count, " blocks for cam ", id);
    if(!count){
        LOG_ERROR("No object found Error stopping cam");
        success = -2;
        return true;
    }else if (count == 1){
        tmp_x = visibleBlocks[0].x;
        tmp_y = visibleBlocks[0].y;
        tmp_a = visibleBlocks[0].a;
        alignBlocks.push_back(visibleBlocks[0]);
        success = 0;
        LOG_GREEN_INFO("Single Tag detection ", id, " position: { x = ", tmp_x, ", y = ", tmp_y, ", a = ", tmp_a, " }");
        // Return true if the values were successfully extracted
    }
    else{
        success = -2;
        for(size_t max_block = MIN(4,count); max_block > 1; max_block -- ){
            if(findGroupRANSAC2D(visibleBlocks,alignBlocks, max_block)){

                if(max_block !=2){
                    tmp_x = alignBlocks[1].x;
                    tmp_y = alignBlocks[1].y;
                    tmp_a = alignBlocks[1].a;
                }else{
                    tmp_x = alignBlocks[0].x;
                    tmp_y = alignBlocks[0].y;
                    tmp_a = alignBlocks[0].a;
                }
                success = max_block;
                break;
            }else{
                LOG_WARNING("Ransac: Pas de solution à ", max_block);
            }
        }
        if(success < 0){
            tmp_x = visibleBlocks[0].x;
            tmp_y = visibleBlocks[0].y;
            tmp_a = visibleBlocks[0].a;
            alignBlocks.push_back(visibleBlocks[0]);
            success = 1;
            LOG_GREEN_INFO("Going for single tag ", id, " position: { x = ", tmp_x, ", y = ", tmp_y, ", a = ", tmp_a, " }");
        }
    }
    
    if(success < 0){
        LOG_ERROR("ArucoCam::getObjectPos() - No object position found");
    }else{
        //Traitement pour passer dans les coordonnées de la table
        // Décalage pour le centre du robot
        tmp_a = (tmp_a > 0) ? tmp_a - 90 : tmp_a + 90;
        double rad_tmp_a = tmp_a * M_PI / 180.0;
        //LOG_EXTENDED_DEBUG("Position avant correction du décalage : { x = ", tmp_x, ", y = ", tmp_y, ", a = ", tmp_a, " }");
        //LOG_EXTENDED_DEBUG("Décalage appliqué : { sin = ", OFFSET_STOCK * mult_param * sin(rad_tmp_a), ", cos = ", OFFSET_STOCK * mult_param * cos(rad_tmp_a), " }");
        const double off_s = 85; // Augmenter pour se rapprocher
        tmp_x += OFFSET_CAM_X - (OFFSET_STOCK - off_s) * cos(rad_tmp_a);
        tmp_y += OFFSET_CAM_Y + OFFSET_CLAW_Y - (OFFSET_STOCK - off_s) * sin(rad_tmp_a);
        //LOG_EXTENDED_DEBUG("Position après correction du décalage : { x = ", tmp_x, ", y = ", tmp_y, ", a = ", tmp_a, " }");


        //projection dans le repère de la table:
        double a_rad = (a) * M_PI / 180.0;
        double cos_a = cos(a_rad);
        double sin_a = sin(a_rad);
        x += tmp_x * cos_a - tmp_y * sin_a;
        y += tmp_x * sin_a + tmp_y * cos_a;
        a += tmp_a;
        LOG_GREEN_INFO("Tag detection ", id, " position: { x = ", x, ", y = ", y, ", a = ", a, " }");
    }
    return true;
}

bool ArucoCam::ToObjectSweep(bool* order, json& data, double &x, double &y, double &a, double &dist_balayage, int& success){
    LOG_DEBUG("=== ENTER ToObjectSweep ===");

    if (!data.contains("objects") || data["objects"].is_null() || !data["objects"].is_object()) {
        LOG_ERROR("ToObjectSweep() - invalid objects");
        success = -1;
        return true;
    }

    success = -2;

    std::vector<block_t> blocks;
    auto& objects = data["objects"];

    // =========================
    // 1. CAMERA → ROBOT
    // =========================
    for (auto& [key, list] : objects.items()) {
        for (auto& obj : list) {

            double raw_x = obj.value("x", 0.0);
            double raw_y = obj.value("y", 0.0);
            double raw_a = obj.value("a", 0.0);
            std::string label = obj.value("label", "");

            double a_rad = -raw_a * M_PI / 180.0;
            double c = cos(a_rad);
            double s = sin(a_rad);

            blocks.push_back(block_t{
                .x = -(raw_x * c - raw_y * s),
                .y = -(raw_x * s + raw_y * c),
                .a = -raw_a,
                .color = (label == "Blue")
            });
        }
    }

    int count = blocks.size();
    LOG_DEBUG("Sweep: Found", count, "blocks");

    if (count == 0) {
        LOG_WARNING("No blocks detected");
        success = -2;
        return true;
    }

    success = count;

    // =========================
    // 2. CENTRE DES BLOCS
    // =========================
    double mean_x = 0, mean_y = 0;
    for (const auto& b : blocks) {
        mean_x += b.x;
        mean_y += b.y;
    }
    mean_x /= count;
    mean_y /= count;

    LOG_DEBUG("Mean -> x:", mean_x, " y:", mean_y);

    // =========================
    // 3. AXE PRINCIPAL (theta)
    // =========================
    double max_dist = 0;
    block_t b1, b2;

    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            double dx = blocks[i].x - blocks[j].x;
            double dy = blocks[i].y - blocks[j].y;
            double d = sqrt(dx*dx + dy*dy);

            if (d > max_dist) {
                max_dist = d;
                b1 = blocks[i];
                b2 = blocks[j];
            }
        }
    }

    double theta = atan2(b2.y - b1.y, b2.x - b1.x);

    LOG_DEBUG("theta(rad):", theta*180/M_PI);

    // =========================
    // 4. TRI PROJECTION (axe theta)
    // =========================
    std::vector<std::pair<block_t,double>> projected;

    double ct = cos(theta);
    double st = sin(theta);

    for (const auto& b : blocks) {
        double p = b.x * ct + b.y * st;
        projected.push_back({b, p});
    }

    std::sort(projected.begin(), projected.end(),
        [](const auto& a, const auto& b) {
            return a.second < b.second;
        });

    // couleur ordre
    int n = projected.size();
    int start = std::max(0, n - 4);

    LOG_DEBUG("Selecting blocks from", start);

    for (int i = 0; i < n - start; i++) {
        order[i] = projected[start + i].first.color;
    }

    // =========================
    // 5. DISTANCE BALAYAGE
    // =========================
    dist_balayage = std::max(0.0, max_dist - 50.0 * (count - 1));
    LOG_DEBUG("dist_balayage:", dist_balayage);

    // =========================
    // 6. CENTRE + OFFSET CAM
    // =========================
    mean_x += OFFSET_CAM_X;
    mean_y += OFFSET_CAM_Y;

    LOG_DEBUG("Corrected mean x = ", mean_x," / y = ", mean_y);

    // =========================
    // 7. ORIENTATION ROBOT (choix stable)
    // =========================
    auto norm = [](double a){
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    };

    double a1 = norm(a + 90 + theta*180/M_PI);
    double a2 = norm(a + 270.0 + theta*180/M_PI);

    double chosen = (fabs(norm(a1 - a)) < fabs(norm(a2 - a))) ? a1 : a2;

    LOG_DEBUG("chosen angle:", chosen);

    // =========================
    // 8. PROJECTION TABLE (PROPRE)
    // =========================
    double ar = a * M_PI / 180.0;
    double ca = cos(ar);
    double sa = sin(ar);

    x += mean_x * ca - mean_y * sa;
    y += mean_x * sa + mean_y * ca;

    // =========================
    // 9. RECUL 270mm SUR AXE CHOSEN
    // =========================
    double r = chosen * M_PI / 180.0;

    x -= 270.0 * cos(r);
    y -= 270.0 * sin(r);

    // =========================
    // 10. UPDATE ANGLE
    // =========================
    a = chosen;

    LOG_DEBUG("FINAL -> x:", x, " y:", y, " a:", a);
    LOG_DEBUG("=== EXIT ToObjectSweep ===");

    return true;
}

/*
    Get the most isolated object to take
*/
bool ArucoCam::ToIsolatedObject(json& data, double & x, double & y, double & a, bool& success){
    int count = 0;
    if (!data.contains("objects") || data["objects"].is_null() || !data["objects"].is_object()) {
        LOG_ERROR("ArucoCam::ToIsolatedObject() - invalid or missing 'objects'");
        success = false;
        return true;
    }
    auto& objects = data["objects"];
    std::vector<block_t> possible;
    for (auto& [key, list] : objects.items()) {
        for (auto& obj : list) {
            
            // On convertit vers le repère robot 
            double a_tag_rad = -1 * obj.value("a",0.0) * M_PI / 180.0;
            double sin_tag = sin(a_tag_rad);
            double cos_tag = cos(a_tag_rad);
            double x_tmp = obj.value("x", 0.0);
            double y_tmp = obj.value("y", 0.0);
            possible.push_back(block_t{
                .x = -1 * (x_tmp* cos_tag - y_tmp * sin_tag),
                .y= -1 * (x_tmp* sin_tag + y_tmp * cos_tag),
                .a = -1 * obj.value("a",0.0),
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
    LOG_GREEN_INFO("Isolated on the other side is ", possible[count-1].color, " at ( ",possible[count-1].x,", ",possible[3].y,")");
    x += possible[0].x;
    y += possible[0].y;
    a += possible[0].a;

    success = true;
    return true;
}

bool ArucoCam::getBestIsolatedObject(double & x, double & y, double & a, bool& success){
    json data;
    int data_success;
    success = false;
    if(getObjectData(data, data_success)){
        if(data_success){
            success = false;
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

bool ArucoCam::getObjectPos(double & x, double & y, double & a, int& success){
    json data;
    int data_success;
    if(getObjectData(data, data_success)){
        if(!data_success){
            success = -1;
            return true;
        }
    }else{
        return false;
    }
    ToObjectPos(data, x, y, a, success);    
    return true;
}

json ArucoCam::getObjectPosition_json(){
    double x = 0;
    double y = 0;
    double a = 0;
    int sucess = -1;
    while(!getObjectPos(x,y,a,sucess)){
        continue;
    }
    return json{
        {"x", x},
        {"y", y},
        {"a", a},
        {"Nombres objets", sucess}};
}

json ArucoCam::getRobotPosition_json(){
    double x = 0;
    double y = 0;
    double a = 0;
    bool sucess = false;
    while(!getRobotPos(x,y,a,sucess)){
        continue;
    }
    return json{
        {"x", x},
        {"y", y},
        {"a", a}};
}

bool ArucoCam::getObjectInfoColors(bool* order, double & x, double & y, double & a, int& success){
    json data;
    int data_success;
    if(getObjectData(data, data_success)){
        if(data_success){
            success = -1;
            return true;
        }
    }else{
        return false;
    }

    if(ToObjectPos(data, x, y, a, success)){// Can not return False so first If is useless
        if(success > 0){
            if(ToObjectColor(order, success)){ // Can not return false either
                return true; // exec finished 
            }else return false; // exec unfinished
        }else return true; // exec finished on fail
    }else return false; // exec unfinished

}

bool ArucoCam::getObjectForSweep(bool* order, double & x, double & y, double & a, int& success, double dist_balayage){
    json data;
    int data_success;

    if(getObjectData(data, data_success)){
        if(data_success){
            success = -1;
            return true;
        }
    }else{
        return false;
    }

    return ToObjectSweep(order, data, x, y, a, dist_balayage, success);
}

void ArucoCam::start() {
    if (id < 0) return;
    json response;
    if (restAPI_GET(url, "/start", response)){
        status = true;
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " started");
    }
}

void ArucoCam::stop() {
    if (id < 0) return;
    json response;
    if (restAPI_GET(url, "/stop", response)){
        status = false;
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " stopped");
    }
}

void ArucoCam::reset_tracking() {
    if (id < 0) return;
    json response;
    if (restAPI_GET(url, "/reset_tracking", response)){
        status = true;
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " reset tracking");
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