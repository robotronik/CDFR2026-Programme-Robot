#include "actions/actionInterface.hpp"
#include "utils/logger.hpp"
#include "actions/functions.h"

class GatherAction : public ActionInterface
{
    public:
        GatherAction();
        typedef enum
        {
            FSM_GATHER_NAV,
            FSM_GATHER_MOVE,
            FSM_GATHER_COLLECT
        } InActionState_t;

        InActionState_t getGatherStockState(){
            return gatherStockState;
        };

        void setGatherStockState(InActionState_t state){
            gatherStockState = state;
        };

        int chooseNextStock();
        bool chooseStockStrategy(int& stockNum, int& stockOffset);

        int getBestStockPositionOff(int stockNum, position_t fromPos);
        void setStockAsRemoved(int num);

        ReturnFSM_t FSM_run() override;
        void reset() override;
        position_t bestMove() override;

    private:
        InActionState_t gatherStockState = FSM_GATHER_NAV;
        int offset = 0; // Offset is direction to take the stock from (0-7)
        position_t stockPos;
        position_t stockOff;
        position_t targetPos;
};