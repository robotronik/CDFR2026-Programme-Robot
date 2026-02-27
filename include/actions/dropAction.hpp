#include "actions/actionInterface.hpp"
#include "defs/constante.h"
#include "defs/tableState.hpp"
#include "utils/logger.hpp"
#include "navigation/navigation.h"
#include "actions/functions.h"
#include "actions/strats.hpp"

class DropAction : public ActionInterface
{
    public:
        DropAction();
        typedef enum
        {
            FSM_DROP_NAV,
            FSM_DROP_DROP,
        } InActionState_t;

        InActionState_t getDropState(){
            return dropState;
        };

        void setDropState(InActionState_t state){
            dropState = state;
        };
        position_t getBestDropZonePosition(int dropzoneNum, position_t fromPos);
        int GetBestDropZone(position_t fromPos);
        void setDropzoneState(int dropzoneNum, TableState::dropzone_state_t state);
        void setDropzoneAsError(int dropzoneNum);
        
        void reset() override;
        position_t bestMove() override;
        ReturnFSM_t FSM_run() override;

    private:
        InActionState_t dropState = FSM_DROP_NAV;
        position_t dropzonePos;
        position_t targetPos;
};