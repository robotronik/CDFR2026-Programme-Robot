#pragma once
#include "actions/VirtualAction.hpp"
#include "actions/VirtualStrategy.hpp"
//#include "db/ActionDurationDB.hpp"
#include "defs/tableState.hpp"

class Robot;

class ActionFSM{
    public:
        ActionFSM(Robot* robot, TableState* tableState);
        ~ActionFSM();
        void Reset();
        bool RunFSM();

        bool isEndlessMode = false;

    private:
        Robot* robotControl = nullptr;
        TableState* tableState = nullptr;
        /***** FUNCTIONS  *******/
        // Pull the strategy currently selected on the robot; drop the running
        // action if it changed.
        void syncStrategy();
        // Determine and return the highest-priority action to run
        VirtualAction* SetBestAction();

        /************  CURRENT STRATEGY ************/
        // The strategy is not owned by the FSM (only referenced): it may be
        // shared/recreated elsewhere (e.g. chosen from the robot's colour/
        // strategy switch).
        VirtualStrategy* currentStrategy = nullptr;

        // Action currently running. It belongs to currentStrategy (bestAction()
        // or tempAction()): the FSM does not own it and must never free it.
        VirtualAction* currentAction = nullptr;

        /************  ACTION DURATIONS ************/
        // Records, for each action carried to completion (ReturnFSM::Done),
        // the time it actually took to run.
        //ActionDurationDB durationDB{ACTION_DB_PATH};
        // Instant (_millis()) when currentAction started.
        unsigned long actionStartTime = 0;
};