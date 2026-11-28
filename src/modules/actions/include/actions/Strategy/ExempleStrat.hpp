#pragma once
#include "actions/VirtualStrategy.hpp"
#include "actions/VirtualAction.hpp"

#include "navigation/driveControl.h"
#include "defs/tableState.hpp"
#include "actions/Robot.hpp"
/*
    ExempleStrat
    ------------
    Example "strategy" in the VirtualStrategy sense: unlike a single
    stateful action, a VirtualStrategy exposes a set of possible actions,
    each weighted, and picks the most interesting/priority one on each call to
    bestAction().

    - each candidate action can tell whether it is playable right now via
      available() (cf. VirtualAction::available()),
    - the weighting (fixed at construction) ranks actions that would all be
      available at the same time.
*/
class ExempleStrat : public VirtualStrategy {
public:
    ExempleStrat(TableState* ts, Robot* rc);
    ~ExempleStrat() override = default;

    /*
        Select, among possible_actions, the action whose score (weighting *
        action->available()) is highest among the available ones
        (available() >= 0), and return a NON-OWNING pointer to it: it stays in
        the pool and is still owned by the strategy, so it can be re-offered.

        Returns nullptr if the strategy is stopped (stop()) or if no action in
        the pool is available.
    */
    VirtualAction* bestAction() override;
    /*
        Temporary ("wait") action to run when no action in the pool is
        available. The strategy keeps ownership: the FSM only gets a pointer
        and must never free it.
    */
    VirtualAction* tempAction() override;

    /* Refill the pool of possible actions (e.g. call it on the FSM Reset(), or
       when the strategy has used up all its actions and must be replayed from
       the start).

       WARNING: the previous actions are destroyed. Call ActionFSM::Reset()
       right after, otherwise the FSM keeps a currentAction pointing to a freed
       action. */
    void reset();

    // Function to handle the strategy
    position_t StratStartingPos() override;
    position_t StratFinishingPos() override;

private:
    // (Re)build the possible_actions pool for the current strategy
    void buildPossibleActions();
    // Temporary action owned by the strategy: it is kept out of the pool (cf.
    // reset()) and never leaves it; the FSM only gets a pointer via
    // tempAction().
    std::unique_ptr<VirtualAction> waitAction;
    TableState* tableStatus;
    Robot* robot;
};