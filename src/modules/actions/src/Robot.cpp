#include "actions/Robot.hpp"

#include "actions/SensorControl.hpp"
#include "actions/Strategy/ExempleStrat.hpp"
#include "actions/VirtualStrategy.hpp"
#include "i2c/Arduino.hpp"
#include "lidar/Lidar.hpp"
#include "mat/mat.hpp"
#include "navigation/navigation.h"
#include "utils/logger.hpp"
#include "vision/Cam.hpp"
#include <cmath>

#include "defs/tableState.hpp"
#include "lidar/lidarAnalize.h"
#include "navigation/pathfind.h"
#include "utils/utils.h"

#ifndef __CROSS_COMPILE_ARM__
    // Desktop build: no real lidar / I2C hardware.
#define DISABLE_LIDAR
#define EMULATE_I2C
#endif

Robot::Robot()
    : arduino_(std::make_unique<Arduino>()),
      sensor_(arduino_.get()),
      drive(),
      actuators(arduino_.get(), &drive),
      wall1(idxWall1, position_t{-70, 120, 120}, &actuators),
      wall2(idxWall2, position_t{0, 0, 0}, &actuators),
      wall3(idxWall3, position_t{0, 0, 0}, &actuators),
      wall4(idxWall4, position_t{0, 0, 0}, &actuators),
      gate1(idxGate1, position_t{0, 0, 0}, &actuators),
      gate2(idxGate2, position_t{0, 0, 0}, &actuators) {
    wallTable[0] = &wall2;
    wallTable[1] = &gate1;
}

void Robot::setTableState(TableState* tableState) {
    tableState_ = tableState;

    // Navigation needs the table; build it now that the table exists.
    navigation_ = std::make_unique<Navigation>(&drive, tableState_, &sensor_.cam(), &calibrationAge);
    // The gates run a navigation sequence directly, so give them the instance.
    gate1.setNavigation(navigation_.get());
    gate2.setNavigation(navigation_.get());

    initStrategies();
}

Robot::~Robot() = default;

Lidar* Robot::lidar() {
    return &sensor_.lidar();
}
Cam* Robot::cam() {
    return &sensor_.cam();
}
Mat* Robot::mat() {
    return &sensor_.mat();
}

bool Robot::homeActuators() {
    actuators.homeActuators();
    bool allHomed = true;
    for (Wall* wall : wallTable) {
        if (!wall->home()) {
            allHomed = false;
        }
    }
    return allHomed;
}

void Robot::initStrategies() {
    strategies.clear();
    strategies.push_back(std::make_unique<ExempleStrat>(tableState_, this));
    currentStrategy = strategies.back().get();
}

std::string Robot::strategyName() const {
    return currentStrategy ? currentStrategy->getNom() : std::string();
}

std::vector<std::string> Robot::strategyNames() const {
    std::vector<std::string> names;
    for (const auto& candidate : strategies)
        if (candidate)
            names.push_back(candidate->getNom());
    return names;
}

bool Robot::hasStrategy(const std::string& name) const {
    for (const auto& candidate : strategies)
        if (candidate && candidate->getNom() == name)
            return true;
    return false;
}

void Robot::startMatchHardware() {
    if (hardwareReady_)
        return;

    actuators.enableActuators();
    // The wheels start at the strategy's starting position: set both the
    // odometry position and the navigation target there so the first move
    // starts from the right pose.
    const position_t startPos = currentStrategy->StratStartingPos();
    drive.setCoordinates(startPos);
    if (navigation_)
        navigation_->goTo(startPos);
    homeActuators();
    sensor_.lidar().startSpin();
    sensor_.cam().start();

    hardwareReady_ = true;
}

void Robot::stopMatchHardware() {
    drive.disable();
    actuators.disableActuators();
    sensor_.lidar().stopSpin();
    sensor_.cam().stop();
    hardwareReady_ = false;
}

void Robot::switchTeamSide(ColorTeam color) {
    if (color == ColorTeam::None) return;
    if (currentState_ && *currentState_ == MainState::Run) return;
    if (color == colorTeam) return;

    LOG_INFO("Color switch detected");
    colorTeam = color;

    switch (color) {
    case ColorTeam::Blue:
        LOG_INFO("Switching to BLUE");
        arduino_->RGB_Blinking(0, 0, 255);
        break;
    case ColorTeam::Yellow:
        LOG_INFO("Switching to YELLOW");
        arduino_->RGB_Blinking(255, 56, 0);
        break;
    default:
        break;
    }

    const position_t pos = currentStrategy->StratStartingPos();
    LOG_INFO("Strategy '", currentStrategy->getNom(),
             "', start: x=", pos.x, " y=", pos.y, " a=", pos.a);
}

void Robot::switchStrategy(const std::string& newStrategy) {
    if (currentState_ && *currentState_ == MainState::Run) return;

    if (currentStrategy && newStrategy == currentStrategy->getNom()) {
        LOG_INFO("Strategy '", newStrategy, "' already active");
        return;
    }

    VirtualStrategy* selectedStrategy = nullptr;
    for (const auto& candidate : strategies)
        if (candidate && candidate->getNom() == newStrategy)
            selectedStrategy = candidate.get();
    if (!selectedStrategy) return;

    LOG_INFO("Strategy switch detected");
    currentStrategy = selectedStrategy;
}

void Robot::requestColor(ColorTeam color) {
    std::lock_guard<std::mutex> lock(requestMutex_);
    requestedColor_ = color;
    hasColorRequest_ = true;
}

void Robot::requestStrategy(const std::string& strategy) {
    std::lock_guard<std::mutex> lock(requestMutex_);
    requestedStrategy_ = strategy;
    hasStrategyRequest_ = true;
}

void Robot::updateTeamStrat() {
    ColorTeam color = ColorTeam::None;
    bool hasColor = false;
    std::string requestedStrategy;
    bool hasStrategy = false;
    {
        std::lock_guard<std::mutex> lock(requestMutex_);
        hasColor = hasColorRequest_;
        color = requestedColor_;
        hasColorRequest_ = false;
        hasStrategy = hasStrategyRequest_;
        requestedStrategy = requestedStrategy_;
        hasStrategyRequest_ = false;
    }

    if (hasColor)
        switchTeamSide(color);
    if (hasStrategy)
        switchStrategy(requestedStrategy);
}

bool Robot::startup() {
    arduino_->RGB_Blinking(255, 0, 0); // Red blinking

#ifndef DISABLE_LIDAR
    if (!sensor_.lidar().setup("/dev/ttyAMA0", 256000)) {
        LOG_ERROR("Cannot find the lidar");
        return false;
    }
#endif

    // The mat thread retries in the background until the mat is reachable and is
    // only stopped at exit.
    if (!sensor_.mat().start())
        LOG_ERROR("MAT not connected at init, retrying in the background");

    drive.reset();
    return true;
}

void Robot::shutdown() {
    sensor_.mat().stop();
    sensor_.lidar().Stop();
    sensor_.cam().stop();

#ifndef EMULATE_I2C
    drive.disable();
    arduino_->RGB_Solid(0, 0, 0); // OFF

    for (int i = 0; i < 60; i++) {
        if (actuators.homeActuators())
            break;
        delay(100);
    }
    actuators.disableActuators();
#endif
}

void Robot::update() {
    drive.update();
    // The odometry prior helps the feature localiser; the marker localiser
    // ignores it.
    sensor_.cam().setPrior(drive.position);

    if (currentState_ && (*currentState_ == MainState::Init || *currentState_ == MainState::Fin))
        return;

#ifndef DISABLE_LIDAR
    pollLidar();
#endif
    pollMat();
}

void Robot::pollLidar() {
    static position_t pos_opponent = OPPONENT_POS_UNKNOWN;
    const bool isDataValid = (fabs(drive.velocity.a) <= 45.0) ||
                             (position_distance(drive.position, pos_opponent) < 1000);

    if (!sensor_.lidar().getData())
        return;

    convertAngularToAxial(sensor_.lidar().data, sensor_.lidar().count, drive.position, 150);
    pathfind_fill_lidar(&sensor_.lidar(), *tableState_);

    // Only update the opponent position if the robot is not moving too fast, to
    // avoid noise.
    const bool running = currentState_ &&
                         (*currentState_ == MainState::Run || *currentState_ == MainState::Manual);
    if (isDataValid && position_opponentV2(sensor_.lidar().data, sensor_.lidar().count, drive.position, pos_opponent) &&
        running &&
        // No match clock yet (startTime == 0): nothing to wait for. Otherwise
        // ignore the first second of the match to avoid false readings.
        (tableState_->startTime == 0 || _millis() - tableState_->startTime > 1000)) {
        tableState_->pos_opponent.x = pos_opponent.x;
        tableState_->pos_opponent.y = pos_opponent.y;
        opponentInAction(pos_opponent);
    }
}

void Robot::pollMat() {
    // The mat background thread refreshes the table: only read the latest data,
    // without blocking the loop.
    MatTableData matData;
    const bool mastReachable = sensor_.mat().latest(matData);
    if (mastReachable)
        tableState_->updateMapStatus(matData);
    tableState_->mastStatus = mastReachable;
}

void Robot::opponentInAction(position_t /*position*/) {}
