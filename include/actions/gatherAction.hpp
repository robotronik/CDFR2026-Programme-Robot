#include "actions/actionInterface.hpp"
#include "defs/constante.h"
#include "defs/tableState.hpp"
class GatherAction : public ActionInterface
{
    public:
        GatherAction();
        typedef enum
        {
            FSM_GATHER_NAV,
            FSM_GATHER_MOVE,
            FSM_GATHER_COLLECT,
            FSM_GATHER_COLLECTED
        } InActionState_t;

        ReturnFSM_t FSM_run() override;
        position_t bestMove() override;
        void reset() override;
    private:
        InActionState_t gatherStockState = FSM_GATHER_NAV;
        int offset = 0; // Offset is direction to take the stock from (0-7)
};