#include "restAPI/restAPI.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ifaddrs.h>
#include <arpa/inet.h>
#include <net/if.h>

#include "utils/logger.hpp"
#include "main.hpp" //for static variables
#include "defs/tableState.hpp"
#include "lidar/lidarAnalize.h" //for static variable
#include "navigation/navigation.h"
#include "navigation/astar.h" //for static variable
#include "actions/functions.h" //for state machine functions

#include "restAPI/crow.hpp"
#include "utils/json.hpp" // For handling JSON
using json = nlohmann::json;

#define API_PORT 80

crow::response readHtmlFile(const std::string& path);
std::string getContentType(const std::string& path);

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

crow::SimpleApp app;

void StartAPIServer(){
    LOG_INFO("Starting API Server on port ", API_PORT);

    // ------------------------------- HTML Pages -------------------------------

    // Define a simple route for the root endpoint
    CROW_ROUTE(app, "/")
    ([](){
        return readHtmlFile("html/index.html");
    });

    // Define a simple route for the display page
    CROW_ROUTE(app, "/display")
    ([](){
        return readHtmlFile("html/display.html");
    });

    // Define a simple route for the lidar page
    CROW_ROUTE(app, "/lidar")
    ([](){
        return readHtmlFile("html/lidar.html");
    });

    // Define a simple route for the lidar page
    CROW_ROUTE(app, "/logs")
    ([](){
        return readHtmlFile("html/logs.html");
    });

    // Define a simple route for the navbar
    CROW_ROUTE(app, "/navbar.html")
    ([](){
        return readHtmlFile("html/navbar.html");
    });

    // Define a simple route for robot page
    CROW_ROUTE(app, "/robot")
    ([](){
        return readHtmlFile("html/robot.html");
    });

    // Define a simple route for the control page
    CROW_ROUTE(app, "/control")
    ([](){
        return readHtmlFile("html/control.html");
    });

    // ------------------------------- GET Routes -------------------------------

    // Define a route for a simple GET request that returns the status
    CROW_ROUTE(app, "/get_status")
    ([](){
        json response;
        response["status"] = currentState;

        return crow::response(response.dump(4));
    });

    // Define a route for a simple GET request that returns the position
    CROW_ROUTE(app, "/get_pos")
    ([](){
        json response;
        response["pos"] = (position_t)drive.position;

        return crow::response(response.dump(4));
    });

    // Define a route for a simple GET request that returns the table status
    CROW_ROUTE(app, "/get_table")
    ([](){
        json response;
        response["table"] = tableStatus;
        response["score"] = tableStatus.getScore();
        return crow::response(response.dump(4));
    });

    // Define a route for a simple GET request that returns all of the information
    CROW_ROUTE(app, "/get_global")
    ([](){
        json response;
        response["status"] = currentState;
        response["table"] = tableStatus;
        response["score"] = tableStatus.getScore();
        
        json limitedAbsoluteLidarData = json::array();

        // Add the first lidar.count elements to the new array
        for (int i = 0; i < lidar.count; ++i) {
            limitedAbsoluteLidarData.push_back({{"x", lidar.data[i].x}, {"y", lidar.data[i].y}});
        }
        response["lidar"] = limitedAbsoluteLidarData;
        json current_navigation_path;
        navigation_path_json(current_navigation_path);
        response["navigation"] = current_navigation_path;
        
        response["target_pos"] = (position_t)drive.target;
        response["pos"] = (position_t)drive.position;
        // Costmap
        response["costmap"] = astar_get_costmap_json();

        return crow::response(response.dump());
    });

    // Define a route for an simple GET request that returns the lidar data
    CROW_ROUTE(app, "/get_lidar")
    ([](){
        json response;
        
        json limitedLidarData = json::array();

        // Add the first lidar_count elements to the new array
        for (int i = 0; i < lidar.count; ++i) {
            limitedLidarData.push_back(lidar.data[i]);
        }
        response["data"] = limitedLidarData;
        return crow::response(response.dump());
    });

    // Define a route for an simple GET request that returns the general data
    CROW_ROUTE(app, "/get_robot")
    ([](){
        json response;
        response["status"] = currentState;
        response["team"] = tableStatus.colorTeam;
        response["score"] = tableStatus.getScore();
        response["time"] = (tableStatus.startTime == 0) ? 0 : (_millis() - tableStatus.startTime);
        response["strategy"] = tableStatus.strategy;
        response["runid"] =  log_asserv()->getLogID();
        return crow::response(response.dump(4));
    });

    // Define a route to return the local IP address(es)
    CROW_ROUTE(app, "/get_ip")
    ([](){
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
    ([](){
        //std::string test = "\x1b[1;31mTESTTAPI\x1b[0m\x1b[2;37mFaint Gray\x1b[0m\x1b[3;34mItalic Blue\x1b[0m\x1b[4;32mUnderlined Green\x1b[0m\x1b[7;30;47mReverse Black on White\x1b[0m\n\ntest";
        std::string test = log_main()->getLogToRobotScreen();
        json response;
        response["log"] = test;
        return crow::response(response.dump(4));
    });

    // Define a route for an simple GET request that returns all of the information from the arduino
    CROW_ROUTE(app, "/get_arduino")
    ([](){
        json response;
        response["message"] = "Failure to get Arduino data!";
        json stepper_pos = json::array();
        for (int i = 1; i <= 4; i++){
            int32_t val ;
            if (!arduino.getStepper(val, i)) return crow::response(400, response.dump(4));
            stepper_pos.push_back({i, val});
        }
        json sensor_state = json::array();
        for (int i = 1; i <= 6; i++){
            bool val ;
            if (!arduino.readSensor(i, val)) return crow::response(400, response.dump(4));
            sensor_state.push_back({i, val});
        }
        response["steppers_pos"] = stepper_pos;
        response["sensors_state"] = sensor_state;
        response["message"] = "Success to fetch Arduino data";
        return crow::response(response.dump(4));
    });

    CROW_ROUTE(app, "/get_costmap")
    ([](){
        json response;
        response["costmap"] = astar_get_costmap_json();
        return crow::response(response.dump());
    });

    // ------------------------------- POST Routes -------------------------------

    // Define a route for a POST request that accepts JSON data and responds with a message
    CROW_ROUTE(app, "/post_status").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        // Parse the incoming JSON
        LOG_INFO("Request body is " + req.body);
        auto req_data = json::parse(req.body);

        // Extract fields
        main_State_t req_state = req_data["state"];

        if (req_state < 0 || req_state > 6){
            // Denies the POST resquest
            json response;
            response["message"] = "Requested state out of borders!";

            // Return the response as JSON
            return crow::response(400, response.dump(4));
        }

        // Apply the post method
        nextState = req_state;

        // Create a response JSON
        json response;
        response["message"] = "Successfull";

        // Return the response as JSON
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request that accepts a color as JSON data (0:None, 1:Blue, 2:Yellow)
    CROW_ROUTE(app, "/set_color").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);
        colorTeam_t req_color = req_data["color"];

        if (req_color < 0 || req_color > 2){
            json response;
            response["message"] = "Invalid Request Color";
            return crow::response(400, response.dump(4));
        }
        if (currentState == RUN){
            json response;
            response["message"] = "Cannot change the color in the current state";
            return crow::response(400, response.dump(4));
        }

        switchTeamSide(req_color);

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request that accepts a strategy to apply
    CROW_ROUTE(app, "/set_strat").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);
        int req_strat = req_data["strat"];

        if (req_strat < 1 || req_strat > 4){
            json response;
            response["message"] = "Invalid Strategy Request";
            return crow::response(400, response.dump(4));
        }
        if (currentState == RUN){
            json response;
            response["message"] = "Cannot change the strategy in the current state";
            return crow::response(400, response.dump(4));
        }

        switchStrategy(req_strat);

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request that sets the manual control mode
    CROW_ROUTE(app, "/set_manual_control_mode").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);
        bool req_value = req_data["value"];
        if(req_value && currentState != WAITSTART){
            json response;
            response["message"] = "Cannot enter manual mode when not in WAITSTART";
            return crow::response(400, response.dump(4));
        }
        else if (!req_value && currentState != MANUAL){
            json response;
            response["message"] = "Cannot exit manual mode when not in MANUAL";
            return crow::response(400, response.dump(4));
        }

        //Apply the value
        manual_ctrl = req_value;

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request that sets the robot position coordinates
    CROW_ROUTE(app, "/set_coordinates").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);

        if (currentState != MANUAL){
            json response;
            response["message"] = "Cannot change the coordinates when not in MANUAL mode";
            return crow::response(400, response.dump(4));
        }

        double req_x_value = drive.position.x;
        if (req_data.contains("x"))
            req_x_value = req_data["x"];
            
        double req_y_value = drive.position.y;
        if (req_data.contains("y"))
            req_y_value = req_data["y"];

        double req_a_value = drive.position.a;
        if (req_data.contains("a"))
            req_a_value = req_data["a"];

        //Apply the values
        position_t pos = {req_x_value, req_y_value, req_a_value};
        drive.setCoordinates(pos);
        navigationGoTo(pos, false, false);

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request that sets a target position
    CROW_ROUTE(app, "/set_target_coordinates").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);

        if (currentState != MANUAL){
            json response;
            response["message"] = "Cannot change the coordinates when not in MANUAL mode";
            return crow::response(400, response.dump(4));
        }

        double req_x_value = drive.position.x;
        if (req_data.contains("x"))
            req_x_value = req_data["x"];
            
        double req_y_value = drive.position.y;
        if (req_data.contains("y"))
            req_y_value = req_data["y"];

        double req_a_value = drive.position.a;

        // Apply the values
        if (req_data.contains("a")){
            double req_a_value = req_data["a"];
            LOG_INFO("Manual ctrl : Requested set_target_coordinates, x=", req_x_value, " y=", req_y_value, " a=", req_a_value);
            position_t pos = {req_x_value, req_y_value, req_a_value};
            navigationGoTo(pos, true, false);
        }
        else{
            LOG_INFO("Manual ctrl : Requested set_target_coordinates, x=", req_x_value, " y=", req_y_value);
            position_t pos = {req_x_value, req_y_value, req_a_value};
            navigationGoTo(pos, false, false);
        }

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request that sets a target position using Astarts
    CROW_ROUTE(app, "/set_target_coordinates_Astart").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);

        if (currentState != MANUAL){
            json response;
            response["message"] = "Cannot change the coordinates when not in MANUAL mode";
            return crow::response(400, response.dump(4));
        }

        double req_x_value = drive.position.x;
        if (req_data.contains("x"))
            req_x_value = req_data["x"];
            
        double req_y_value = drive.position.y;
        if (req_data.contains("y"))
            req_y_value = req_data["y"];

        double req_a_value = drive.position.a;

        // Apply the values
        if (req_data.contains("a")){
            LOG_INFO("Manual ctrl : Requested set_target_coordinates_Astart, x=", req_x_value, " y=", req_y_value, " a=", req_a_value);
            position_t pos = {req_x_value, req_y_value, req_a_value};
            navigationGoTo(pos, true, true);
        }
        else{
            LOG_INFO("Manual ctrl : Requested set_target_coordinates_Astart, x=", req_x_value, " y=", req_y_value);
            position_t pos = {req_x_value, req_y_value, req_a_value};

            navigationGoTo(pos, true, true);
        }

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request that move by a certain amount in mm
    CROW_ROUTE(app, "/set_move").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);

        if (currentState != MANUAL){
            json response;
            response["message"] = "Cannot move when not in MANUAL mode";
            return crow::response(400, response.dump(4));
        }

        double req_value = req_data["value"];

        double newXvalue = drive.position.x + cos(drive.position.a * DEG_TO_RAD) * req_value;
        double newYvalue = drive.position.y + sin(drive.position.a * DEG_TO_RAD) * req_value;

        LOG_INFO("Manual ctrl : Requested set_move, value=", req_value);

        // Apply the value
        position_t pos = {newXvalue, newYvalue, drive.position.a};
        navigationGoTo(pos, false);

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request to rotate by a certain amount in degrees
    CROW_ROUTE(app, "/set_rotate").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);

        if (currentState != MANUAL){
            json response;
            response["message"] = "Cannot rotate when not in MANUAL mode";
            return crow::response(400, response.dump(4));
        }

        int req_value = req_data["value"];

        // Apply the value
        // TODO
        position_t pos = {drive.position.x, drive.position.y, drive.position.a + req_value};
        navigationGoTo(pos, true);

        LOG_INFO("Manual ctrl : Requested set_rotate, value=", req_value);

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request to rotate a servo to a certain value
    CROW_ROUTE(app, "/set_servo").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);

        if (currentState != MANUAL){
            json response;
            response["message"] = "Cannot rotate servo when not in MANUAL mode";
            return crow::response(400, response.dump(4));
        }

        int req_value = req_data["value"];
        int req_id = req_data["id"];
        int req_speed = req_data["speed"];

        //Apply the value
        if (req_speed != 0)
            arduino.moveServoSpeed(req_id, req_value, req_speed);
        else
            arduino.moveServo(req_id, req_value);

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a POST request to rotate a stepper to a certain value
    CROW_ROUTE(app, "/set_stepper").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);

        if (currentState != MANUAL){
            json response;
            response["message"] = "Cannot rotate stepper when not in MANUAL mode";
            return crow::response(400, response.dump(4));
        }

        int req_value = req_data["value"];
        int req_id = req_data["id"];

        //Apply the value
        arduino.moveStepper(req_value, req_id);

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a PORT request to test action functions
    CROW_ROUTE(app, "/test_action").methods(crow::HTTPMethod::POST)([](const crow::request& req){
        auto req_data = json::parse(req.body);

        if (currentState != MANUAL){
            json response;
            response["message"] = "Cannot test actions when not in MANUAL mode";
            return crow::response(400, response.dump(4));
        }
        if (manual_currentFunc != nullptr){
            json response;
            response["message"] = "Another action is currently running, cannot execute the requested action";
            return crow::response(400, response.dump(4));
        }

        std::string req_value = req_data["value"];

        // Apply the value
        if      (req_value == "closeClaws")       manual_currentFunc = closeClaws;
        else if (req_value == "openClaws")        manual_currentFunc = openClaws;
        else if (req_value == "resetSpinClaws")   manual_currentFunc = resetSpinClaws;
        else if (req_value == "spinAllClaws")     manual_currentFunc = spinAllClaws;
        else if (req_value == "lowerClaws")       manual_currentFunc = lowerClaws;
        else if (req_value == "raiseClaws")       manual_currentFunc = raiseClaws;
        else if (req_value == "raiseLittleClaws") manual_currentFunc = raiseLittleClaws;
        else {
            json response;
            response["message"] = "Invalid action requested";
            return crow::response(400, response.dump(4));
        }

        json response;
        response["message"] = "Successfull";
        return crow::response(response.dump(4));
    });

    // Define a route for a stop request
    CROW_ROUTE(app, "/stop").methods(crow::HTTPMethod::POST)([](const crow::request& req){

        json response;
        response["message"] = "Successfull";

        // Apply the value
        //exit_requested = true;
        nextState = FIN;

        return crow::response(response.dump(4));
    });

    // ------------------------------- Routes for serving files -------------------------------

    // Route for serving SVG and PNG files
    CROW_ROUTE(app, "/assets/<string>")
    .methods(crow::HTTPMethod::GET)([](const std::string& filename) {
        std::ifstream file("html/assets/" + filename, std::ios::binary);
        
        if (!file) {
            return crow::response(404, "File not found");
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        crow::response res{buffer.str()};
        res.set_header("Cache-Control", "public, max-age=3600");
        res.set_header("Content-Type", getContentType(filename));
        return res;
    });

    // Route for serving css files
    CROW_ROUTE(app, "/css/<string>")
    .methods(crow::HTTPMethod::GET)([](const std::string& filename) {
        return readHtmlFile("html/css/" + filename);
    });
    // Route for serving js files
    CROW_ROUTE(app, "/js/<string>")
    .methods(crow::HTTPMethod::GET)([](const std::string& filename) {
        return readHtmlFile("html/js/" + filename);
    });


    // Route for serving the favicon ico files
    CROW_ROUTE(app, "/favicon.ico") ([](){
        std::ifstream file("html/favicon.ico", std::ios::binary);
        if (!file) {
            return crow::response(404, "File not found");
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        crow::response res{buffer.str()};
        res.set_header("Cache-Control", "public, max-age=3600");
        res.set_header("Content-Type", "image/x-icon");
        return res;
    });


    // Set the port and run the app with 2 threads
    app.loglevel(crow::LogLevel::Warning);
    app.signal_clear().port(API_PORT).concurrency(2).run();
}

void StopAPIServer(){
    app.stop();
    LOG_INFO("Stopped API Server");
}

void TestAPIServer(){
    // Sets some variable to display them statically

    drive.position.a = 15;
    drive.position.x = 100;
    drive.position.y = 100;

    tableStatus.pos_opponent.x = 300;
    tableStatus.pos_opponent.y = 300;
}


// Function to read an HTML file and return its content as a crow::response
crow::response readHtmlFile(const std::string& path) {
    std::ifstream file(path);
    
    if (!file) {  // Check if file exists and is open
        return crow::response(404, "File not found");  // Return a 404 response if file is missing
    }

    std::stringstream buffer;
    buffer << file.rdbuf();  // Read the file into the buffer

    crow::response res{buffer.str()};
    res.set_header("Content-Type", getContentType(path));  // Set correct Content-Type header
    res.set_header("Cache-Control", "public, max-age=3600");
    return res;
}

// Helper function to determine Content-Type based on file extension
std::string getContentType(const std::string& path) {
    std::string extension = path.substr(path.find_last_of(".") + 1);
    if (extension == "css") {
        return "text/css";
    } else if (extension == "js") {
        return "application/javascript";
    } else if (extension == "svg") {
        return "image/svg+xml";
    } else if (extension == "png") {
        return "image/png";
    } else if (extension == "html"){
        return "text/html";
    } else {
        return "text/plain";  // Default to plain text
    }
}