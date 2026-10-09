#include "defs/tableState.hpp"
#include "mat/mat.hpp"

TableState::TableState(DriveControl* drive){
    this->drive = drive;
    pos_opponent.x = 3000; pos_opponent.y = 0; //si on detect pas l'adversaire, on se mettrait en slow mode proche de 0,0
    colorTeam = NONE;
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
    int c_y = colorTeam == BLUE ? (900 + h/2) : (-900 - h/2);
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

// Serialize tableState
void to_json(json& j, const TableState& ts) {
    j = json{
        {"pos_opponent", ts.pos_opponent},
        {"startTime", ts.startTime},
        {"colorTeam", ts.colorTeam},
        {"strategy", ts.strategy}
    };
}

void TableState::updateMapStatus(){
    const MatTableData data = getMatTableData();

    elements = data.elements;

    // Le mat voit le robot adverse (tag de couleur opposée) : sa position fait
    // foi.
    if (!data.opponentVisible) {
        return; // on conserve la dernière position connue
    }
    pos_opponent.x = data.opponentX;
    pos_opponent.y = data.opponentY;
    pos_opponent.a = data.opponentA;
}