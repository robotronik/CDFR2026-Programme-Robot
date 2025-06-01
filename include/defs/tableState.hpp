#pragma once
#include "defs/structs.hpp"
#include "defs/constante.h"
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
    robot_t robot;

    int strategy;

    /* data Winter is comming */
};

// Serialize tableState
void to_json(json& j, const TableState& ts);