#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "vision/ArucoCam.hpp"
#include "vision/ArucoDetector.hpp"
#include "utils/logger.hpp"

#define SCAN_FAIL_FRAMES_NUM 2
#define SCAN_DONE_FRAMES_NUM 10

namespace {

// Capture resolution, matching the previous Python service defaults.
constexpr int kCameraWidth = 1280;
constexpr int kCameraHeight = 800;

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

// Calibration tags: physical size (mm) and known global position (mm).
struct CalibrationTag {
    double size;
    double globalX;
    double globalY;
};

const std::map<int, CalibrationTag> kCalibrationTags = {
    {33, {100.0,    0.0,    0.0}}, // testing
    {20, {100.0, -400.0, -900.0}}, // Top-right
    {21, {100.0, -400.0,  900.0}}, // Top-left
    {22, {100.0,  400.0, -900.0}}, // Bottom-left
    {23, {100.0,  400.0,  900.0}}, // Bottom-right
    { 0, {100.0,    0.0,    0.0}}, // test
};

// Game objects: physical size (mm) and team label.
struct GameObjectTag {
    double size;
    const char* label;
};

const std::map<int, GameObjectTag> kGameObjectTags = {
    {36, {30.0, "Blue"}},
    {47, {30.0, "Yellow"}},
};

// Objects detected higher than this (mm) are ignored (side/elevated tags).
constexpr double kObjectMaxHeight = 200.0;

double nowSeconds() {
    return std::chrono::duration<double>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

double normalizeAngle(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

// Yaw (rotation about Z) of the extrinsic 'zyx' Euler decomposition, matching
// scipy's Rotation.as_euler('zyx')[0] used by the previous Python service.
double yawFromRotation(const cv::Matx33d& R) {
    return std::atan2(-R(0, 1), R(0, 0)) * kRadToDeg;
}

// Camera position expressed in the marker frame: -R^T * t.
cv::Vec3d cameraPositionFromPose(const cv::Matx33d& R, const cv::Vec3d& tvec) {
    const cv::Vec3d rotated = R.t() * tvec;
    return cv::Vec3d{-rotated[0], -rotated[1], -rotated[2]};
}

} // namespace

ArucoCam::ArucoCam(int cam_number, const char* calibration_file_path) {
    id = cam_number;
    if (id < 0) {
        LOG_INFO("Emulating ArucoCam");
        return;
    }

    if (!detector_.loadCalibration(calibration_file_path)) {
        LOG_ERROR("ArucoCam ", id, " failed to load calibration from ", calibration_file_path);
    }

    std::map<int, double> markerSizes;
    for (const auto& [markerId, tag] : kCalibrationTags) {
        markerSizes[markerId] = tag.size;
    }
    for (const auto& [markerId, tag] : kGameObjectTags) {
        markerSizes.emplace(markerId, tag.size);
    }
    detector_.setMarkerSizes(markerSizes);
}

ArucoCam::~ArucoCam() {
    stop();
}

bool ArucoCam::start() {
    if (id < 0) return false;
    if (running_.load()) return true;

    if (!detector_.isCameraOpen() && !detector_.initCamera(id, kCameraWidth, kCameraHeight)) {
        LOG_ERROR("ArucoCam ", id, " failed to open camera");
        return false;
    }

    reset_tracking(); // Fresh counters, matching the previous /start behaviour
    running_.store(true);
    worker_ = std::thread(&ArucoCam::workerLoop, this);
    LOG_GREEN_INFO("ArucoCam ", id, " started");
    return true;
}

void ArucoCam::stop() {
    if (id < 0) return;
    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
    detector_.releaseCamera();
    LOG_EXTENDED_DEBUG("ArucoCam ", id, " stopped");
}

void ArucoCam::reset_tracking() {
    if (id < 0) return;
    std::lock_guard<std::mutex> lock(stateMutex_);
    state_ = State{};
    LOG_EXTENDED_DEBUG("ArucoCam ", id, " reset tracking");
}

void ArucoCam::workerLoop() {
    while (running_.load()) {
        cv::Mat frame;
        if (!detector_.captureFrame(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        const std::vector<vision::DetectionResult> detections = detector_.detect(frame);

        std::lock_guard<std::mutex> lock(stateMutex_);
        if (processDetections(detections)) {
            state_.successFrames++;
        } else {
            state_.failedFrames++;
        }
    }
}

bool ArucoCam::processDetections(const std::vector<vision::DetectionResult>& detections) {
    json detectedObjects = json::object();
    bool found = false;

    for (const auto& detection : detections) {
        const bool isCalibrationTag = kCalibrationTags.count(detection.id) > 0;
        const bool isGameObject = kGameObjectTags.count(detection.id) > 0;
        if (!isCalibrationTag && !isGameObject) continue;
        if (!detection.hasPose) continue;

        cv::Matx33d rotation;
        cv::Rodrigues(detection.rvec, rotation);
        const cv::Vec3d cameraPosition = cameraPositionFromPose(rotation, detection.tvec);
        const double yaw = yawFromRotation(rotation);

        if (isCalibrationTag) {
            const CalibrationTag& tag = kCalibrationTags.at(detection.id);
            const double x = tag.globalX - cameraPosition[1];
            const double y = tag.globalY + cameraPosition[0];
            const double z = cameraPosition[2];
            const double a = normalizeAngle(-yaw + 180.0);
            addAveragePosition(x, y, z, a);
            found = true;
        }

        if (isGameObject) {
            if (cameraPosition[2] >= kObjectMaxHeight) continue;
            const GameObjectTag& tag = kGameObjectTags.at(detection.id);
            detectedObjects[std::to_string(detection.id)].push_back(json{
                {"label", tag.label},
                {"x", -cameraPosition[1]},
                {"y", cameraPosition[0]},
                {"z", cameraPosition[2]},
                {"a", normalizeAngle(-yaw + 180.0)},
                {"last_seen", nowSeconds()},
            });
        }
    }

    state_.objects = std::move(detectedObjects);
    return found;
}

void ArucoCam::addAveragePosition(double x, double y, double z, double a) {
    if (state_.successFrames == 0 || !state_.hasPosition) {
        state_.x = x;
        state_.y = y;
        state_.z = z;
        state_.a = a;
        state_.hasPosition = true;
        return;
    }

    const double newRatio = 1.0 / (state_.successFrames + 1.0);
    const double oldRatio = 1.0 - newRatio;
    state_.x = state_.x * oldRatio + x * newRatio;
    state_.y = state_.y * oldRatio + y * newRatio;
    state_.z = state_.z * oldRatio + z * newRatio;

    const double y1 = std::sin(state_.a * kDegToRad) * oldRatio;
    const double x1 = std::cos(state_.a * kDegToRad) * oldRatio;
    const double y2 = std::sin(a * kDegToRad) * newRatio;
    const double x2 = std::cos(a * kDegToRad) * newRatio;
    state_.a = std::atan2(y1 + y2, x1 + x2) * kRadToDeg;
}

// Returns true when done
bool ArucoCam::getPos(double & x, double & y, double & a, bool& success) {
    success = false;
    if (id < 0) {
        return true;
    }
    if (!running_.load()) { // Camera is not running
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " is not running, will start it now");
        if (!start()) {
            LOG_ERROR("ArucoCam ", id, " could not be started, aborting position fetch");
            waitingForPos_ = false;
            return true;
        }
        waitingForPos_ = true;
        return false;
    }
    if (!waitingForPos_) { // Reset tracking
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " starting tracking");
        reset_tracking();
        waitingForPos_ = true;
        return false;
    }

    int failedFrames = 0;
    int successFrames = 0;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        failedFrames = state_.failedFrames;
        successFrames = state_.successFrames;
        if (failedFrames <= SCAN_FAIL_FRAMES_NUM && successFrames >= SCAN_DONE_FRAMES_NUM) {
            x = state_.x;
            y = state_.y;
            a = state_.a;
            success = true;
        }
    }

    if (failedFrames > SCAN_FAIL_FRAMES_NUM) {
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " has too many failed frames : ", failedFrames);
        waitingForPos_ = false;
        return true;
    }
    if (successFrames < SCAN_DONE_FRAMES_NUM) {
        return false;
    }

    LOG_GREEN_INFO("ArucoCam ", id, " position: { x = ", x, ", y = ", y, ", a = ", a,
                   " } with success frames : ", successFrames, " and failed frames : ", failedFrames);
    waitingForPos_ = false;
    return true;
}

bool ArucoCam::getObjectData(json& objects, int& sucess){
    sucess = -1;
    if (id < 0) {
        return true;
    }
    if (!running_.load()) {
        LOG_EXTENDED_DEBUG("ArucoCam ", id, " is not running, will start it now");
        if (!start()) {
            LOG_ERROR("ArucoCam::getObjectData() - camera ", id, " could not be started");
            return true;
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(stateMutex_);
    objects = json{{"objects", state_.objects}};
    sucess = 0;
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
    for(size_t i =  0; i< alignBlocks.size(); i++){  
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
        const double off_s = 115; // Augmenter pour se rapprocher
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
    bool myBlue = (success == 1); // true si notre équipe est BLEUE // on considère que si success == 1 alors le bloc trouvé est de notre couleur, sinon c'est un bloc ennemi

    if (!data.contains("objects") || data["objects"].is_null() || !data["objects"].is_object()) {
        success = -1; return true;
    }

    std::vector<block_t> blocks;
    auto& objects = data["objects"];

    // 1. Conversion Caméra -> Robot
    for (auto& [key, list] : objects.items()) {
        for (auto& obj : list) {
            double raw_x = obj.value("x", 0.0), raw_y = obj.value("y", 0.0), raw_a = obj.value("a", 0.0);
            double a_rad = -raw_a * M_PI / 180.0;
            blocks.push_back(block_t{
                .x = -(raw_x * cos(a_rad) - raw_y * sin(a_rad)),
                .y = -(raw_x * sin(a_rad) + raw_y * cos(a_rad)),
                .a = -raw_a,
                .color = (obj.value("label", "") == "Blue")
            });
        }
    }

    int count = blocks.size();
    if (count == 0) { success = -2; return true; }
    success = std::min(4, count); // ON RENVOIE LE NOMBRE REEL DE BLOCS

    // 2. Trouver les deux blocs les plus éloignés pour définir l'axe
    double theta = 0;
    double max_dist = 0;

    if (count >= 2) {
        int idx1 = 0, idx2 = 0;
        for (int i = 0; i < count; i++) {
            for (int j = i + 1; j < count; j++) {
                double d = sqrt(pow(blocks[i].x - blocks[j].x, 2) + pow(blocks[i].y - blocks[j].y, 2));
                if (d > max_dist) { max_dist = d; idx1 = i; idx2 = j; }
            }
        }
        theta = atan2(blocks[idx2].y - blocks[idx1].y, blocks[idx2].x - blocks[idx1].x);
    } else {
        // Cas 1 seul bloc : on prend son orientation propre convertie en radians
        // Attention : vérifie si blocks[0].a est en degrés ou radians dans ton système
        LOG_ERROR("angle block = ", blocks[0].a);
        theta = blocks[0].a * M_PI / 180.0; 
        max_dist = 0; 
    }

    // On s'assure que l'axe pointe vers la DROITE du robot
    if (sin(theta) > 0) theta += M_PI;
    double ct = cos(theta), st = sin(theta);

    // 3. TRI : On projette chaque bloc sur cet axe et on trie du plus à GAUCHE au plus à DROITE
    std::sort(blocks.begin(), blocks.end(), [&](const block_t& A, const block_t& B){
        double projA = A.x * ct + A.y * st;
        double projB = B.x * ct + B.y * st;
        return projA < projB; // Plus petite projection (gauche) en premier
    });

    for(int i = 0; i < count - 1; i++){
        double dx = blocks[i+1].x - blocks[i].x;
        double dy = blocks[i+1].y - blocks[i].y;
        double d = sqrt(dx*dx + dy*dy);

        if(d > 100.0){
            LOG_WARNING("dist = ", d);
            int enemyBlocks = 0;
            for(const auto& b : blocks){
                if(b.color != myBlue) enemyBlocks++;
            }

            // abandon seulement si < 2 blocs adverses
            if(enemyBlocks < 2){
                success = -3;
                return true;
            }

        LOG_WARNING("Far blocks but enough enemy blocks -> continue");
        break;
    }
    }
    // 4. MAPPING : On remplit les pinces dans l'ordre de détection
    // On reset tout à true par sécurité
    //for(int i = 0; i < 4; i++) order[i] = true; 
    
    for (int i = 0; i < success; i++) {
        order[4 - blocks.size() + i] = blocks[i].color;
    }

    for (int i = 0; i < 4; i++) LOG_DEBUG("Pince", i, "assignée au bloc rang", i, (order[i] ? "[BLEU]" : "[JAUNE]"));

    // 5. Calcul distance balayage et positions (inchangé)
    dist_balayage = std::max(0.0, max_dist - 50.0 * (count - 1));
    double mean_x = 0, mean_y = 0;
    for (const auto& b : blocks) { mean_x += b.x; mean_y += b.y; }
    mean_x = (mean_x / count) + OFFSET_CAM_X;
    mean_y = (mean_y / count) + OFFSET_CAM_Y;

    auto norm = [](double ang){ while(ang > 180) ang-=360; while(ang < -180) ang+=360; return ang; };
    double a1 = norm(a + 90 + theta*180/M_PI), a2 = norm(a + 270 + theta*180/M_PI);
    double chosen = (fabs(norm(a1 - a)) < fabs(norm(a2 - a))) ? a1 : a2;

    double ar = a * M_PI / 180.0;
    x += mean_x * cos(ar) - mean_y * sin(ar);
    y += mean_x * sin(ar) + mean_y * cos(ar);
    x -= 260.0 * cos(chosen * M_PI / 180.0);
    y -= 260.0 * sin(chosen * M_PI / 180.0);
    a = chosen;

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
    LOG_GREEN_INFO("Isolated on the other side is ", possible[count-1].color, " at ( ",possible[count-1].x,", ",possible[count-1].y,")");
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

bool ArucoCam::getObjectForSweep(bool* order, double & x, double & y, double & a, int& success, double& dist_balayage){
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

