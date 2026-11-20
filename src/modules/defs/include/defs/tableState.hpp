#pragma once
#include "defs/structs.hpp"
#include "defs/matTypes.hpp"
#include "defs/constante.h"
#include <utils/json.hpp>
#include <vector>

using json = nlohmann::json;

class RobotControl;

class TableState
{
    public:

        explicit TableState(RobotControl* robot);
        ~TableState();

        void reset();
        int getScore();
        
        /* common data */
        position_t pos_opponent;
        unsigned long startTime;
        ColorTeam colorTeam;
        std::string strategy;

        /* Cam calibration related*/
        int calibrationAge; // Age of calibration by Camera if exist
        void resetCalibrationAge(){ calibrationAge = 0;}

        bool mastStatus = false; // True while the mast is reachable and publishing
        void updateMapStatus(const MatTableData& data); // update of tableState relative to data recived by mast

        bool m_isPointInsideRectangle(float px, float py, float cx, float cy, float w, float h);
        // Arrival area (throne room): where the main robot must finish the match.
        bool isRobotInArrivalArea(position_t position);

        /* data the Legend of Camelot */
        // Objets de jeu vus par le mat, mémorisés par updateMapStatus().
        // Les stratégies (ex: GoToPositionAction) les utilisent pour viser
        // l'élément le plus proche du robot.
        std::vector<MatGameElement> elements;
        quarry_t quarry[10];
        bool remainQuarry();
        int isQuarryAvailable(int idx);

        wallZone_t wallZone[WALL_ZONE_COUNT];
        towerZone_t towerZone[TOWER_ZONE_COUNT];

        bool remainEmptyWall();
        bool remainEmptyTower();
        bool remainEmptyConstructionZone();
        int needTower();
        int needWall();
        bool needGate();

    private:
        RobotControl* robot;
        void initQuarry();
        void initConstructionZones();
};

// Serialize tableState
void to_json(json& j, const TableState& ts);
