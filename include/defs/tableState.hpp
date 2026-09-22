#pragma once
#include "defs/structs.hpp"
#include <utils/json.hpp>

using json = nlohmann::json;

class TableState
{
    public:

        TableState();
        ~TableState();

        void reset();
        int getScore();
        
        /* common data */
        position_t pos_opponent;
        unsigned long startTime;
        colorTeam_t colorTeam;
        int strategy;

        /* Cam calibration related*/
        int calibrationAge; // Age of calibration by Camera if exist
        void resetCalibrationAge(){ calibrationAge = 0;}

        bool mastStatus = false; // Status of mast if exist
        void updateMapStatus(); // update of tableState relative to data recived by mast

        /* data the Legend of Camelot */


};

// Serialize tableState
void to_json(json& j, const TableState& ts);
