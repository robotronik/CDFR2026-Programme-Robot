#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include "drive_interface.h" // for position_t
#include "vision/ArucoDetector.hpp"
#include "vision/ArucoLocalizer.hpp"
#include "vision/CamLocalizer.hpp"
#include "vision/FeaturesLocalizer.hpp"

// The camera is mounted on the robot at this offset, in millimetres and degrees.
#define OFFSET_CAM_X 129 // Offset of the camera in mm on the x axis
#define OFFSET_CAM_Y 4.5 // Offset of the camera in mm on the y axis
#define OFFSET_CAM_A 0 // Offset angle of the camera in degrees

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
    // Change to have a position_t
    // and height as steps, 1,2,3 and a vertical bool
    // make it a general struct in structs.hpp to use as well in mat logic
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

// Captures frames on its own thread and localises the camera on the field.
// Both localisers run on every frame - ArUco landmark tags and mapped ground
// features - so their results are always available; USE_ARUCO_LOCALISATION only
// selects which one getLocalisation() reports. ArUco detection also feeds the
// game elements and the preview.
class Cam {
public:
    Cam(int camNumber, const char* calibrationFilePath, const char* mapFilePath);
    ~Cam();

    Cam(const Cam&) = delete;
    Cam& operator=(const Cam&) = delete;

    // Starts/stops the capture thread. A negative camNumber emulates a camera
    // that is never started.
    void start();
    void stop();

    // Odometry estimate of the robot's pose, used by the feature localiser to
    // restrict its search. Ignored by the marker localiser.
    void setPrior(const position_t& robotPose);

    // Latest camera localisation on the table, from the localiser selected by
    // USE_ARUCO_LOCALISATION. Returns true when one is known. This is the
    // camera's pose; use cameraToRobot() for the robot's.
    bool getLocalisation(position_t& cameraPose) const;

    // JPEG-encoded copy of the latest captured frame, with the detected markers
    // outlined and labelled. Returns false when no frame has been captured yet.
    bool getPreview(std::vector<uchar>& jpeg) const;

    // Game elements seen in the latest frame, placed on the table from the
    // given camera pose.
    std::vector<GameElement> getGameElements(const position_t& cameraPose) const;

    // Places one detection on the table as the centre of its game element cube,
    // from the given camera pose. Returns false when the detection is not a
    // game element carrying a usable pose.
    static bool gameElementFromTag(const vision::DetectionResult& detection,
                                   const position_t& cameraPose,
                                   GameElement& element);

private:
    void workerLoop();

    int id_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;

    // Owns the capture device and detects the markers for the preview and the
    // game elements.
    vision::ArucoDetector detector_;
    // Both localisers run on every frame; USE_ARUCO_LOCALISATION picks which
    // one getLocalisation() reports.
    vision::ArucoLocalizer arucoLocalizer_;
    vision::FeaturesLocalizer featuresLocalizer_;

    mutable std::mutex mutex_;
    std::vector<vision::DetectionResult> detections_;
    cv::Mat frame_;
    bool hasLocalisation_ = false;
    position_t localisation_ = {0.0, 0.0, 0.0};

    bool hasPrior_ = false;
    position_t priorRobot_ = {0.0, 0.0, 0.0};
};
