#pragma once
#include "defs/structs.hpp"
#include "navigation/driveControl.h"
#include <utils/json.hpp>

using json = nlohmann::json;

class TableState
{
    public:

        TableState(DriveControl* drive);
        ~TableState();

        void reset();
        int getScore();
        
        /* common data */
        position_t pos_opponent;
        unsigned long startTime;
        colorTeam_t colorTeam;
        std::string strategy;

        /* Cam calibration related*/
        int calibrationAge; // Age of calibration by Camera if exist
        void resetCalibrationAge(){ calibrationAge = 0;}

        bool mastStatus = false; // Status of mast if exist
        void updateMapStatus(); // update of tableState relative to data recived by mast

        bool m_isPointInsideRectangle(float px, float py, float cx, float cy, float w, float h);
        bool isRobotInArrivalZone(position_t position);

        /* data the Legend of Camelot */

    private:
        DriveControl* drive;
};

// Serialize tableState
void to_json(json& j, const TableState& ts);
