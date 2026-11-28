#pragma once
#include "defs/structs.hpp"
#include "defs/matTypes.hpp"
#include "defs/constante.h"
#include <utils/json.hpp>
#include <vector>

using json = nlohmann::json;

class TableState {
    public:
        explicit TableState(const ColorTeam* teamColor);
        ~TableState();

        void reset();
        int getScore(const position_t& robotPos);

        /* Team colour, owned by the Robot and kept in sync through a pointer. */
        ColorTeam teamColor() const { return teamColor_ ? *teamColor_ : ColorTeam::None; }

        /* common data */
        position_t pos_opponent;
        unsigned long startTime;

        bool mastStatus = false; // True while the mast is reachable and publishing
        void updateMapStatus(const MatTableData& data); // update of tableState relative to data recived by mast

        bool isPointInsideRectangle(float px, float py, float cx, float cy, float w, float h);
        // Arrival area (throne room): where the main robot must finish the match.
        bool isRobotInArrivalArea(position_t position);

        /* data the Legend of Camelot */
        // Game objects seen by the mat, cached by updateMapStatus().
        // Strategies use them to target the element closest to the robot.
        std::vector<MatGameElement> elements;
        quarry_t quarry[10];
        bool hasAvailableQuarry();
        bool hasQuarryAvailable(int idx);

        wallZone_t wallZone[WALL_ZONE_COUNT];
        towerZone_t towerZone[TOWER_ZONE_COUNT];

        bool hasEmptyWall();
        bool hasEmptyTower();
        bool hasEmptyConstructionZone();
        int countMissingTowers();
        int countMissingWalls();
        bool isGateNeeded();

    private:
        const ColorTeam* teamColor_;
        void initQuarry();
        void initConstructionZones();
};

// Serialize tableState
void to_json(json& j, const TableState& ts);
