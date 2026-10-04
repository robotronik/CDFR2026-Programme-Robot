#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include "defs/structs.hpp" // for position_t
#include "vision/ArucoLocalizer.hpp"

// The camera is mounted on the robot at this offset, in millimetres and degrees.
#define OFFSET_CAM_X 129 // Offset of the camera in mm on the x axis
#define OFFSET_CAM_Y 4.5 // Offset of the camera in mm on the y axis
#define OFFSET_CAM_A 0 // Offset angle of the camera in degrees

// Camera mounting used to lift a detection into the table frame: how high the
// camera sits above the table and how far it tilts down. Provisional values, to
// be calibrated for the robot.
#define CAMERA_HEIGHT_MM 233.2
#define CAMERA_PITCH_DEG 45.0

// Game elements are 110 mm cubes. Each face carries an ArUco id 13 marker whose
// inner pattern is 80 mm, drawn with a white margin around it (the tag, 100 mm,
// only matters for rendering). `GAME_ELEMENT_SIDE_MM` is the cube's physical
// side, used to move from a marker's centre to the cube's centre;
// `GAME_ELEMENT_TAG_MM` is the marker's own side, which sets the scale of its
// estimated pose.
#define GAME_ELEMENT_SIDE_MM 110.0
#define GAME_ELEMENT_TAG_MM 80.0

// A game element is a marker with ArUco id 13. Its pose is on the table, in
// millimetres, with its Euler angles in degrees. The yaw reference follows the
// marker's own axes (a marker lying flat, oriented like the field tags, reads
// about 90 degrees).
struct GameElement {
    // TODO 
    // Change definition to have a position_t
    // and a level (1,2 or 3)
    // and a bool is_vertical
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

// The camera and the robot are not the same point: the camera sits at
// (OFFSET_CAM_X, OFFSET_CAM_Y) with heading OFFSET_CAM_A in the robot frame.
// These convert a table pose between the two frames.
position_t cameraToRobot(const position_t& cameraPose);
position_t robotToCamera(const position_t& robotPose);

// Captures frames on its own thread and detects ArUco markers. It answers two
// questions: where the camera is on the table, and where the game elements are.
class ArucoCam {
public:
    ArucoCam(int camNumber, const char* calibrationFilePath);
    ~ArucoCam();

    ArucoCam(const ArucoCam&) = delete;
    ArucoCam& operator=(const ArucoCam&) = delete;

    // Starts/stops the capture thread. A negative camNumber emulates a camera
    // that is never started.
    void start();
    void stop();
    bool isEmulated() const { return id_ < 0; }

    // Latest camera localisation on the table. Returns true when one is known.
    // This is the camera's pose; use cameraToRobot() for the robot's.
    bool getLocalisation(position_t& cameraPose) const;

    // Game elements seen in the latest frame, placed on the table from the
    // given camera pose.
    std::vector<GameElement> getGameElements(const position_t& cameraPose) const;

    // Game elements of the latest frame, placed on the table from the latest
    // known camera pose.
    std::vector<GameElement> getGameElements() const;

    // Places one detection on the table as the centre of its game element cube,
    // from the given camera pose. Returns false when the detection is not a
    // game element carrying a usable pose.
    static bool gameElementFromTag(const vision::DetectionResult& detection,
                                   const position_t& cameraPose,
                                   GameElement& element);

private:
    void workerLoop();
    // Lifts the game elements of the latest frame into the table frame using
    // the given camera pose. Called by the worker thread with mutex_ held.
    void updateGameElements(const position_t& cameraPose);

    int id_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;
    vision::ArucoLocalizer localizer_;

    mutable std::mutex mutex_;
    std::vector<vision::DetectionResult> detections_;
    bool hasLocalisation_ = false;
    position_t localisation_ = {0.0, 0.0, 0.0};
    std::vector<GameElement> gameElements_;
};
