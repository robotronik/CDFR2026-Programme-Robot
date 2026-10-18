#include "restAPI/restAPI.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <net/if.h>

#include "utils/logger.hpp"
#include "defs/mainState.hpp"
#include "defs/tableState.hpp"
#include "i2c/Arduino.hpp"
#include "lidar/Lidar.hpp"
#include "lidar/lidarAnalize.h" //for static variable
#include "navigation/navigation.h"
#include "navigation/astar.h" //for static variable
#include "actions/VirtualStrategy.hpp" // for the strategy list
#include "vision/Cam.hpp" //for vision fuction
#include "restAPI/manual_mode.h" // for manual mode functions

#include <crow.h>
#include "utils/json.hpp" // For handling JSON
using json = nlohmann::json;

#define API_PORT 80

RestAPI::RestAPI(main_State_t* currentState,
                 main_State_t* nextState,
                 DriveControl* drive,
                 TableState* tableStatus,
                 Arduino* arduino,
                 Lidar* lidar,
                 Cam* cam,
                 Navigation* navigation,
                 std::vector<VirtualStrategy*>* strategies)
    : currentState(currentState),
      nextState(nextState),
      drive(drive),
      tableStatus(tableStatus),
      arduino(arduino),
      lidar(lidar),
      cam(cam),
      navigation(navigation),
      strategies(strategies){
}

bool RestAPI::consumeColorRequest(colorTeam_t& color){
    if (!hasColorRequest) return false;
    color = requestedColor;
    hasColorRequest = false;
    return true;
}

bool RestAPI::consumeStrategyRequest(std::string& strategy){
    if (!hasStrategyRequest) return false;
    strategy = requestedStrategy;
    hasStrategyRequest = false;
    return true;
}

static crow::response serveFile(const std::string& path);
static std::string getContentType(const std::string& path);

static std::vector<std::string> getLocalIPv4Addresses(){
    std::vector<std::string> ips;
    ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) == -1 || !ifaddr) {
        return ips;
    }

    for (ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!ifa->ifa_addr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET) continue;
        if (ifa->ifa_flags & IFF_LOOPBACK) continue;

        char buffer[INET_ADDRSTRLEN];
        void* addr_ptr = &reinterpret_cast<sockaddr_in*>(ifa->ifa_addr)->sin_addr;
        if (inet_ntop(AF_INET, addr_ptr, buffer, sizeof(buffer)) != nullptr) {
            ips.emplace_back(buffer);
        }
    }

    freeifaddrs(ifaddr);
    return ips;
}

static crow::SimpleApp app;

// Error reply with a JSON body, so every failure path looks the same.
static crow::response jsonError(int code, const std::string& message){
    json response;
    response["message"] = message;
    return crow::response(code, response.dump(4));
}

static crow::response jsonMessage(const std::string& message){
    json response;
    response["message"] = message;
    return crow::response(response.dump(4));
}

// Parses the request body into `out`; returns false when it is not valid JSON.
static bool parseBody(const crow::request& req, json& out){
    out = json::parse(req.body, nullptr, false);
    return !out.is_discarded();
}

// Reads x/y/a from the request, keeping the current value for absent fields.
static position_t readRequestedPosition(const json& req_data, const position_t& current){
    position_t pos = current;
    if (req_data.contains("x")) pos.x = req_data["x"];
    if (req_data.contains("y")) pos.y = req_data["y"];
    if (req_data.contains("a")) pos.a = req_data["a"];
    return pos;
}

static bool isKnownStrategy(const std::vector<VirtualStrategy*>* strategies, const std::string& name){
    if (strategies == nullptr) return false;
    for (VirtualStrategy* strategy : *strategies)
        if (strategy != nullptr && strategy->getNom() == name)
            return true;
    return false;
}

void RestAPI::start(){
    LOG_INFO("Starting API Server on port ", API_PORT);

    // ------------------------------- HTML Pages -------------------------------

    // Define a simple route for the root endpoint
    CROW_ROUTE(app, "/")
    ([this](){
        return serveFile("html/index.html");
    });

    // Define a simple route for the display page
    CROW_ROUTE(app, "/display")
    ([this](){
        return serveFile("html/display.html");
    });

    // Define a simple route for the lidar page
    CROW_ROUTE(app, "/lidar")
    ([this](){
        return serveFile("html/lidar.html");
    });

    // Define a simple route for the lidar page
    CROW_ROUTE(app, "/logs")
    ([this](){
        return serveFile("html/logs.html");
    });

    // Define a simple route for the navbar
    CROW_ROUTE(app, "/navbar.html")
    ([this](){
        return serveFile("html/navbar.html");
    });

    // Define a simple route for robot page
    CROW_ROUTE(app, "/robot")
    ([this](){
        return serveFile("html/robot.html");
    });

    // Define a simple route for the control page
    CROW_ROUTE(app, "/control")
    ([this](){
        return serveFile("html/control.html");
    });

    // Define a simple route for the camera page
    CROW_ROUTE(app, "/camera")
    ([this](){
        return serveFile("html/camera.html");
    });

    // ------------------------------- GET Routes -------------------------------

    // Define a route for a simple GET request that returns the status
    CROW_ROUTE(app, "/get_status")
    ([this](){
        json response;
        response["status"] = *currentState;

        return crow::response(response.dump(4));
    });

    // Define a route for a simple GET request that returns the position
    CROW_ROUTE(app, "/get_pos")
    ([this](){
        json response;
        response["pos"] = (position_t)drive->position;

        return crow::response(response.dump(4));
    });

    // Define a route for a simple GET request that returns the table status
    CROW_ROUTE(app, "/get_table")
    ([this](){
        json response;
        response["table"] = *tableStatus;
        response["score"] = tableStatus->getScore();
        return crow::response(response.dump(4));
    });

    // Define a route for a simple GET request that returns all of the information
    CROW_ROUTE(app, "/get_global")
    ([this](){
        json response;
        response["status"] = *currentState;
        response["table"] = *tableStatus;
        response["score"] = tableStatus->getScore();
        
        json limitedAbsoluteLidarData = json::array();

        // Add the first lidar->count elements to the new array
        for (int i = 0; i < lidar->count; ++i) {
            limitedAbsoluteLidarData.push_back({{"x", lidar->data[i].x}, {"y", lidar->data[i].y}});
        }
        response["lidar"] = limitedAbsoluteLidarData;
        json current_navigation_path;
        navigation->pathJson(current_navigation_path);
        response["navigation"] = current_navigation_path;
        
        response["target_pos"] = (position_t)drive->target;
        response["pos"] = (position_t)drive->position;
        // Costmap
        response["costmap"] = astar_get_costmap_json();

        return crow::response(response.dump());
    });

    // Define a route for an simple GET request that returns the lidar data
    CROW_ROUTE(app, "/get_lidar")
    ([this](){
        json response;
        
        json limitedLidarData = json::array();

        // Add the first lidar_count elements to the new array
        for (int i = 0; i < lidar->count; ++i) {
            limitedLidarData.push_back(lidar->data[i]);
        }
        response["data"] = limitedLidarData;
        return crow::response(response.dump());
    });

    // Define a route for an simple GET request that returns the general data
    CROW_ROUTE(app, "/get_robot")
    ([this](){
        json response;
        response["status"] = *currentState;
        response["team"] = tableStatus->colorTeam;
        response["score"] = tableStatus->getScore();
        response["time"] = (tableStatus->startTime == 0) ? 0 : (_millis() - tableStatus->startTime);
        response["strategy"] = tableStatus->strategy;
        response["runid"] = log_main_get_id();
        return crow::response(response.dump(4));
    });

    // Define a route that returns the names of the available strategies
    CROW_ROUTE(app, "/get_strategies")
    ([this](){
        json strategies_json = json::array();
        if (strategies != nullptr){
            for (VirtualStrategy* strategy : *strategies){
                if (strategy != nullptr)
                    strategies_json.push_back(strategy->getNom());
            }
        }
        json response;
        response["strategies"] = strategies_json;
        return crow::response(response.dump());
    });

    // Define a route to return the local IP address(es)
    CROW_ROUTE(app, "/get_ip")
    ([this](){
        json response;
        auto ips = getLocalIPv4Addresses();
        if (ips.empty()){
            response["ip"] = "127.0.0.1";
            response["ips"] = json::array();
        } else {
            response["ip"] = ips.front();
            response["ips"] = ips;
        }
        return crow::response(response.dump(4));
    });


    CROW_ROUTE(app, "/get_log")
    ([this](){
        //std::string test = "\x1b[1;31mTESTTAPI\x1b[0m\x1b[2;37mFaint Gray\x1b[0m\x1b[3;34mItalic Blue\x1b[0m\x1b[4;32mUnderlined Green\x1b[0m\x1b[7;30;47mReverse Black on White\x1b[0m\n\ntest";
        std::string test = log_main_get_screen();
        json response;
        response["log"] = test;
        return crow::response(response.dump(4));
    });

    // Define a route for an simple GET request that returns all of the information from the arduino
    CROW_ROUTE(app, "/get_arduino")
    ([this](){
        json response;
        response["message"] = "Failure to get Arduino data!";
        json stepper_pos = json::array();
        for (int i = 1; i <= 4; i++){
            int32_t val ;
            if (!arduino->getStepper(val, i)) return crow::response(400, response.dump(4));
            stepper_pos.push_back({i, val});
        }
        json sensor_state = json::array();
        for (int i = 1; i <= 6; i++){
            bool val ;
            if (!arduino->readSensor(i, val)) return crow::response(400, response.dump(4));
            sensor_state.push_back({i, val});
        }
        response["steppers_pos"] = stepper_pos;
        response["sensors_state"] = sensor_state;
        response["message"] = "Success to fetch Arduino data";
        return crow::response(response.dump(4));
    });

    CROW_ROUTE(app, "/get_costmap")
    ([this](){
        json response;
        response["costmap"] = astar_get_costmap_json();
        return crow::response(response.dump());
    });

    // --------------------------- Camera Routes ---------------------------

    // Game elements (ArUco id 13 markers) seen by the camera, as full poses on
    // the table. Placed from the camera's own localisation when it has one,
    // otherwise from the robot's known position.
    CROW_ROUTE(app, "/objects")
    ([this](){
        position_t camera_pos = {0.0, 0.0, 0.0};
        if (!cam->getLocalisation(camera_pos)){
            camera_pos = robotToCamera(drive->position);
        }
        const std::vector<GameElement> elements = cam->getGameElements(camera_pos);
        json objects = json::array();
        for (const GameElement& element : elements){
            objects.push_back(json{{"x", element.x}, {"y", element.y}, {"z", element.z},
                                   {"roll", element.roll}, {"pitch", element.pitch}, {"yaw", element.yaw}});
        }
        json response;
        response["objects"] = objects;
        crow::response res(response.dump());
        res.set_header("Content-Type", "application/json");
        return res;
    });

    // Latest camera localisation, converted to the robot's frame.
    CROW_ROUTE(app, "/position")
    ([this](){
        position_t camera_pos = {0.0, 0.0, 0.0};
        json response;
        if (cam->getLocalisation(camera_pos)){
            const position_t robot_pos = cameraToRobot(camera_pos);
            response["object"] = json{{"x", robot_pos.x}, {"y", robot_pos.y}, {"a", robot_pos.a}};
        }
        else{
            response["object"] = nullptr;
        }
        crow::response res(response.dump());
        res.set_header("Content-Type", "application/json");
        return res;
    });

    // Starts/stops the camera capture thread.
    CROW_ROUTE(app, "/start")
    ([this](){
        cam->start();
        json response;
        response["message"] = "Camera started";
        return crow::response(response.dump());
    });

    CROW_ROUTE(app, "/stop")
    ([this](){
        cam->stop();
        json response;
        response["message"] = "Camera stopped";
        return crow::response(response.dump());
    });

    // Latest captured frame as a JPEG.
    CROW_ROUTE(app, "/preview")
    ([this](){
        std::vector<uchar> jpeg;
        if (!cam->getPreview(jpeg))
            return jsonError(503, "No preview available");
        crow::response res(200, std::string(jpeg.begin(), jpeg.end()));
        res.set_header("Content-Type", "image/jpeg");
        return res;
    });

    // Latest captured frame as a JPEG, without the marker overlay.
    CROW_ROUTE(app, "/preview_raw")
    ([this](){
        std::vector<uchar> jpeg;
        if (!cam->getRawPreview(jpeg))
            return jsonError(503, "No preview available");
        crow::response res(200, std::string(jpeg.begin(), jpeg.end()));
        res.set_header("Content-Type", "image/jpeg");
        return res;
    });

    // Camera settings (exposure compensation, contrast, brightness)
    CROW_ROUTE(app, "/camera_settings").methods(crow::HTTPMethod::GET)
    ([this](){
        json response;
        response["exposure_value"] = cam->getExposureValue();
        response["contrast"] = cam->getContrast();
        response["brightness"] = cam->getBrightness();
        return crow::response(200, response.dump());
    });

    CROW_ROUTE(app, "/camera_settings").methods(crow::HTTPMethod::POST)
    ([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");
        if (req_data.contains("exposure_value") && req_data["exposure_value"].is_number()) {
            cam->setExposureValue(req_data["exposure_value"].get<float>());
        }
        if (req_data.contains("contrast") && req_data["contrast"].is_number()) {
            cam->setContrast(req_data["contrast"].get<float>());
        }
        if (req_data.contains("brightness") && req_data["brightness"].is_number()) {
            cam->setBrightness(req_data["brightness"].get<float>());
        }
        json response;
        response["message"] = "Camera settings updated";
        response["exposure_value"] = cam->getExposureValue();
        response["contrast"] = cam->getContrast();
        response["brightness"] = cam->getBrightness();
        return crow::response(200, response.dump());
    });

    // ------------------------------- POST Routes -------------------------------

    // Define a route for a POST request that accepts JSON data and responds with a message
    CROW_ROUTE(app, "/post_status").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        LOG_INFO("Request body is " + req.body);

        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        main_State_t req_state = req_data["state"];

        // Only the states the API is allowed to select: MANUAL and CALIBRATION
        // are entered by the state machine itself.
        if (req_state < 0 || req_state > TEST)
            return jsonError(400, "Requested state out of borders!");

        *nextState = req_state;
        return jsonMessage("Successful");
    });

    // Define a route for a POST request that accepts a color as JSON data (0:None, 1:Blue, 2:Yellow)
    CROW_ROUTE(app, "/set_color").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        colorTeam_t req_color = req_data["color"];

        if (req_color < 0 || req_color > YELLOW)
            return jsonError(400, "Invalid Request Color");
        if (*currentState == RUN)
            return jsonError(400, "Cannot change the color in the current state");

        requestedColor = req_color;
        hasColorRequest = true;

        return jsonMessage("Successful");
    });

    // Define a route for a POST request that accepts a strategy to apply
    CROW_ROUTE(app, "/set_strat").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        std::string req_strat = req_data["strat"];

        if (!isKnownStrategy(strategies, req_strat))
            return jsonError(400, "Invalid Strategy Request");
        if (*currentState == RUN)
            return jsonError(400, "Cannot change the strategy in the current state");

        requestedStrategy = req_strat;
        hasStrategyRequest = true;

        return jsonMessage("Successful");
    });

    // Define a route for a POST request that sets the manual control mode
    CROW_ROUTE(app, "/set_manual_control_mode").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        bool req_value = req_data["value"];
        if (req_value && *currentState != WAITSTART)
            return jsonError(400, "Cannot enter manual mode when not in WAITSTART");
        if (!req_value && *currentState != MANUAL)
            return jsonError(400, "Cannot exit manual mode when not in MANUAL");

        manual_ctrl = req_value;
        return jsonMessage("Successful");
    });

    // Define a route for a POST request that sets the robot position coordinates
    CROW_ROUTE(app, "/set_coordinates").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        if (*currentState != MANUAL)
            return jsonError(400, "Cannot change the coordinates when not in MANUAL mode");

        const position_t pos = readRequestedPosition(req_data, drive->position);
        drive->setCoordinates(pos);
        navigation->goTo(pos, false);

        return jsonMessage("Successful");
    });

    // Define a route for a POST request that sets a target position
    CROW_ROUTE(app, "/set_target_coordinates").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        if (*currentState != MANUAL)
            return jsonError(400, "Cannot change the coordinates when not in MANUAL mode");

        const position_t pos = readRequestedPosition(req_data, drive->position);
        LOG_INFO("Manual ctrl : Requested set_target_coordinates, x=", pos.x, " y=", pos.y, " a=", pos.a);
        navigation->goTo(pos, false);

        return jsonMessage("Successful");
    });

    // Define a route for a POST request that sets a target position using Astarts
    CROW_ROUTE(app, "/set_target_coordinates_Astart").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        if (*currentState != MANUAL)
            return jsonError(400, "Cannot change the coordinates when not in MANUAL mode");

        const position_t pos = readRequestedPosition(req_data, drive->position);
        LOG_INFO("Manual ctrl : Requested set_target_coordinates_Astart, x=", pos.x, " y=", pos.y, " a=", pos.a);
        navigation->goTo(pos, true);

        return jsonMessage("Successful");
    });

    // Define a route for a POST request that move by a certain amount in mm
    CROW_ROUTE(app, "/set_move").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        if (*currentState != MANUAL)
            return jsonError(400, "Cannot move when not in MANUAL mode");

        double req_value = req_data["value"];

        double newXvalue = drive->position.x + cos(drive->position.a * DEG_TO_RAD) * req_value;
        double newYvalue = drive->position.y + sin(drive->position.a * DEG_TO_RAD) * req_value;

        LOG_INFO("Manual ctrl : Requested set_move, value=", req_value);

        position_t pos = {newXvalue, newYvalue, drive->position.a};
        navigation->goTo(pos);

        return jsonMessage("Successful");
    });

    // Define a route for a POST request to rotate by a certain amount in degrees
    CROW_ROUTE(app, "/set_rotate").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        if (*currentState != MANUAL)
            return jsonError(400, "Cannot rotate when not in MANUAL mode");

        int req_value = req_data["value"];

        // TODO
        position_t pos = {drive->position.x, drive->position.y, drive->position.a + req_value};
        navigation->goTo(pos);

        LOG_INFO("Manual ctrl : Requested set_rotate, value=", req_value);

        return jsonMessage("Successful");
    });

    // Define a route for a POST request to rotate a servo to a certain value
    CROW_ROUTE(app, "/set_servo").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        if (*currentState != MANUAL)
            return jsonError(400, "Cannot rotate servo when not in MANUAL mode");

        int req_value = req_data["value"];
        int req_id = req_data["id"];
        int req_speed = req_data["speed"];

        if (req_speed != 0)
            arduino->moveServoSpeed(req_id, req_value, req_speed);
        else
            arduino->moveServo(req_id, req_value);

        return jsonMessage("Successful");
    });

    // Define a route for a POST request to rotate a stepper to a certain value
    CROW_ROUTE(app, "/set_stepper").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        if (*currentState != MANUAL)
            return jsonError(400, "Cannot rotate stepper when not in MANUAL mode");

        int req_value = req_data["value"];
        int req_id = req_data["id"];

        arduino->moveStepper(req_value, req_id);

        return jsonMessage("Successful");
    });

    // Define a route for a PORT request to test action functions
    CROW_ROUTE(app, "/test_action").methods(crow::HTTPMethod::POST)([this](const crow::request& req){
        json req_data;
        if (!parseBody(req, req_data))
            return jsonError(400, "Invalid JSON");

        if (*currentState != MANUAL)
            return jsonError(400, "Cannot test actions when not in MANUAL mode");
        if (manual_currentFunc != nullptr)
            return jsonError(400, "Another action is currently running, cannot execute the requested action");

        std::string req_value = req_data["value"];

        if (req_value != "Test"){
            // TODO associate each command with the action to test:
            // manual_currentFunc = ...;
            return jsonError(400, "Invalid action requested");
        }

        return jsonMessage("Successful");
    });

    // Define a route for a stop request
    CROW_ROUTE(app, "/stop").methods(crow::HTTPMethod::POST)([this](const crow::request&){

        // Apply the value
        //exit_requested = true;
        *nextState = FIN;

        return jsonMessage("Successful");
    });

    // ------------------------------- Routes for serving files -------------------------------

    // Route for serving SVG and PNG files
    CROW_ROUTE(app, "/assets/<string>")
    .methods(crow::HTTPMethod::GET)([this](const std::string& filename) {
        return serveFile("html/assets/" + filename);
    });

    // Route for serving css files
    CROW_ROUTE(app, "/css/<string>")
    .methods(crow::HTTPMethod::GET)([this](const std::string& filename) {
        return serveFile("html/css/" + filename);
    });
    // Route for serving js files
    CROW_ROUTE(app, "/js/<string>")
    .methods(crow::HTTPMethod::GET)([this](const std::string& filename) {
        return serveFile("html/js/" + filename);
    });


    // Route for serving the favicon ico files
    CROW_ROUTE(app, "/favicon.ico") ([this](){
        return serveFile("html/favicon.ico");
    });


    // Set the port and run the app with 2 threads.
    // signal_clear() stops crow from installing its own SIGINT handler, which
    // would otherwise steal Ctrl-C from the main loop.
    app.loglevel(crow::LogLevel::Warning);
    app.signal_clear().port(API_PORT).concurrency(2).run();
}

void RestAPI::stop(){
    app.stop();
    LOG_INFO("Stopped API Server");
}
// Serves a file from disk with its Content-Type, or a 404. Binary read so that
// images work as well as HTML/CSS/JS.
static crow::response serveFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary);

    if (!file) {
        return crow::response(404, "File not found");
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    crow::response res{buffer.str()};
    res.set_header("Content-Type", getContentType(path));
    res.set_header("Cache-Control", "public, max-age=3600");
    return res;
}

// Content-Type from the file extension.
static std::string getContentType(const std::string& path) {
    std::string extension = path.substr(path.find_last_of(".") + 1);
    if (extension == "css") {
        return "text/css";
    } else if (extension == "js") {
        return "application/javascript";
    } else if (extension == "svg") {
        return "image/svg+xml";
    } else if (extension == "png") {
        return "image/png";
    } else if (extension == "ico") {
        return "image/x-icon";
    } else if (extension == "html"){
        return "text/html";
    } else {
        return "text/plain";  // Default to plain text
    }
}
