#include "defs/tableState.hpp"
#include "defs/constante.h"

TableState::TableState(DriveControl* drive){
    this->drive = drive;
    pos_opponent.x = 3000; pos_opponent.y = 0; //si on detect pas l'adversaire, on se mettrait en slow mode proche de 0,0
    colorTeam = ColorTeam::None;
    strategy = "";
    startTime = 0;
    calibrationAge = 0;
    reset();
}

TableState::~TableState(){}

void TableState::reset(){
    resetCalibrationAge();
    /* reset of specific The legend of Camelot */
    elements.clear();
}

// Function to check if a point (px, py) lies inside the rectangle
bool TableState::m_isPointInsideRectangle(float px, float py, float cx, float cy, float w, float h) {
    float left = cx - w / 2, right = cx + w / 2;
    float bottom = cy - h / 2, top = cy + h / 2;
    return (px >= left && px <= right && py >= bottom && py <= top);
}

bool TableState::isRobotInArrivalZone(position_t position){
    // Returns true if the robot is in the arrival zone
    int robotSmallRadius = 100;
    int w = 450;
    int h = 600;
    int c_x = -550 - w/2;
    int c_y = colorTeam == ColorTeam::Blue ? (900 + h/2) : (-900 - h/2);
    return m_isPointInsideRectangle(position.x, position.y, c_x, c_y, w + 2*robotSmallRadius, h + 2*robotSmallRadius);
}

int TableState::getScore()
{
    int totalScore = 0;
    // TODO, should be "completely inside" and not just "in"
    if (isRobotInArrivalZone((position_t)drive->getPosition()))
        totalScore += 5;
    return totalScore;
}

void TableState::initDropZone(){
    for(int i = 0; i < WALL_ZONE_COUNT; i++) {
        wallZone[i].pos = WALL_POSITION[i];
        wallZone[i].nb = 0;
        wallZone[i].bestConstruction = ConstructionType::Unconstructed;
        wallZone[i].idx = i;
        wallZone[i].colorTeam = i < WALL_ZONE_COUNT / 2 ? ColorTeam::Blue : ColorTeam::Yellow;
    }
    for(int i = 0; i < TOWER_ZONE_COUNT; i++) {
        towerZone[i].pos = TOWER_POSITION[i];
        towerZone[i].nb = 0;
        towerZone[i].bestConstruction = ConstructionType::Unconstructed;
        towerZone[i].idx = i;
        towerZone[i].colorTeam = i < TOWER_ZONE_COUNT / 2 ? ColorTeam::Blue : ColorTeam::Yellow;
    }
}

bool TableState::remainEmptyDrop(){
    return remainEmptyWall() || remainEmptyTower();
}

bool TableState::remainEmptyTower(){
    for(towerZone_t tower : towerZone) {
        if (tower.nb == 0) {
            return true;
        }
    }
    return false;
}

bool TableState::remainEmptyWall(){
    for(wallZone_t wall : wallZone) {
        if (wall.nb == 0) {
            return true;
        }
    }
    return false;
}

int TableState::needTower(){
    int count = 4;
    for(towerZone_t tower : towerZone) {
        if (tower.bestConstruction == ConstructionType::Tower) {
            count--;
        }
    }
    return count;
}

int TableState::needWall(){
    int count = 5;
    for(wallZone_t wall : wallZone) {
        if (wall.bestConstruction == ConstructionType::Wall) {
            count--;
        }
    }
    return count;
}

bool TableState::needDoor(){
    for(wallZone_t wall : wallZone) {
        if (wall.bestConstruction == ConstructionType::Door) {
            return false;
        }
    }
    return true;
}

int TableState::isQuarryAvailable(int idx) {
    return quarry[idx].nb;
}

bool TableState::remainQuarry()
{
    for (int i = 0; i < 10; i++)
    {
        if (isQuarryAvailable(i))
        {
            return true;
        }
    }
    return false;
}
// Serialize tableState
void to_json(json& j, const TableState& ts) {
    j = json{
        {"pos_opponent", ts.pos_opponent},
        {"startTime", ts.startTime},
        {"colorTeam", static_cast<int>(ts.colorTeam)},
        {"strategy", ts.strategy},
        {"mastStatus", ts.mastStatus}
    };
}

void TableState::updateMapStatus(const MatTableData& data){
    elements = data.elements;

    if (!data.opponentVisible) {
        return;
    }
    pos_opponent.x = data.opponentX;
    pos_opponent.y = data.opponentY;
    pos_opponent.a = data.opponentA;
}