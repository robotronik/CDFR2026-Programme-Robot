#pragma once

// Consider using enum class for better type safety
typedef enum
{
    FSM_RETURN_WORKING =0x0,
    FSM_RETURN_DONE    =0x1,
    FSM_RETURN_ERROR   =0x2
} ReturnFSM_t;

class ActionFSM{
public:
    ActionFSM();
    ~ActionFSM();
    void Reset();
    bool RunFSM();

private:
    ReturnFSM_t GatherStock();

    typedef enum
    {
        FSM_ACTION_GATHER,
        FSM_ACTION_NAV_HOME
    } StateRun_t;

    StateRun_t runState = FSM_ACTION_NAV_HOME;

    typedef enum
    {
        FSM_GATHER_NAV,
        FSM_GATHER_COLLECT
    } StateGatherStock_t;

    StateGatherStock_t gatherStockState = FSM_GATHER_NAV;
};