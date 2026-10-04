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

namespace vision {

// The localiser Cam runs, chosen at compile time by USE_ARUCO_LOCALISATION.
// Both expose the same locate() shape, so the call site does not change.
#if USE_ARUCO_LOCALISATION
using LocalizerType = ArucoLocalizer;
#else
using LocalizerType = FeaturesLocalizer;
#endif

} // namespace vision

// Captures frames on its own thread, detects ArUco markers (for game elements
// and the preview) and localises the camera on the field. The localisation
// itself comes from `vision::LocalizerType`: the landmark tags or the mapped
// ground features, switched by USE_ARUCO_LOCALISATION.
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

    // Latest camera localisation on the table. Returns true when one is known.
    // This is the camera's pose; use cameraToRobot() for the robot's.
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
    // game elements, whatever localiser is selected.
    vision::ArucoDetector detector_;
    vision::LocalizerType localizer_;

    mutable std::mutex mutex_;
    std::vector<vision::DetectionResult> detections_;
    cv::Mat frame_;
    bool hasLocalisation_ = false;
    position_t localisation_ = {0.0, 0.0, 0.0};

    bool hasPrior_ = false;
    position_t priorRobot_ = {0.0, 0.0, 0.0};
};
