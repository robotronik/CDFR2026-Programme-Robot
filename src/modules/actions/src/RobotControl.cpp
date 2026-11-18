#include "actions/RobotControl.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

RobotControl::RobotControl(Arduino* arduino)
    : drive(),
      actuators(arduino, &drive),
      wall1(wall1Tab, position_t{-70, 120, 120}, &actuators),
      wall2(wall2Tab, position_t{0, 0, 0}, &actuators),
      wall3(wall3Tab, position_t{0, 0, 0}, &actuators),
      wall4(wall4Tab, position_t{0, 0, 0}, &actuators),
      gate1(3, 2, gate1Tab, position_t{0, 0, 0}, &actuators),
      gate2(5, 4, gate2Tab, position_t{0, 0, 0}, &actuators) {
}

double RobotControl::navigationRadius(double chassisRadius) const {
    double radius = chassisRadius;
    const Wall* faces[] = {&wall1, &wall2, &wall3, &wall4, &gate1, &gate2};
    for (const Wall* face : faces) {
        if (face->stoneCount <= 0) continue;
        const position_t offset = face->getOffset();
        const double angle = offset.a * std::numbers::pi / 180;
        for (double x : {-GAME_ELEMENT_DEPTH_MM / 2, GAME_ELEMENT_DEPTH_MM / 2}) {
            for (double y : {-GAME_ELEMENT_WIDTH_MM / 2, GAME_ELEMENT_WIDTH_MM / 2}) {
                radius = std::max(radius, std::hypot(offset.x + x * std::cos(angle) - y * std::sin(angle),
                                                     offset.y + x * std::sin(angle) + y * std::cos(angle)));
            }
        }
    }
    return radius;
}
