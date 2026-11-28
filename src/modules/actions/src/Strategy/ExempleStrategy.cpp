#include "actions/Strategy/ExempleStrat.hpp"
#include "actions/VirtualAction.hpp"
#include "utils/logger.hpp"
#include "navigation/pathfind.h"
#include "defs/constante.h"

#include "actions/ElementalAction/WaitAction.hpp"
//#include "actions/ElementalAction/CalibrationAction.hpp"
#include "actions/ElementalAction/NavHomeAction.hpp"
#include "actions/ElementalAction/DropStoneAction.hpp"
#include "actions/ElementalAction/TakeStoneAction.hpp"

/*
    ============================================================
    Actions élémentaires de la stratégie d'exemple
    ============================================================
    Plutôt qu'une seule action "à états" qui enchaîne les étapes en dur, chaque étape de la stratégie
    devient ici sa propre VirtualAction, réutilisable et indépendamment
    pondérable dans possible_actions. C'est ExempleStrat::bestAction()
    qui décide de l'ordre en les proposant une par une au fur et à
    mesure qu'elles deviennent la plus prioritaire.
*/

ExempleStrat::ExempleStrat(TableState* ts, Robot* rc) {
    nom = "ExempleStrat";
    tableStatus = ts;
    robot = rc;
    reset();
}

position_t ExempleStrat::StratStartingPos() {
    // Returns the starting position of the robot
    position_t pos = START_POSITION;

    if (robot->colorTeam == ColorTeam::Yellow)
        position_robot_flip(pos);

    return pos;
}

position_t ExempleStrat::StratFinishingPos() {
    // Returns the finishing position of the robot
    position_t pos = FINAL_POSITION;

    if (robot->colorTeam == ColorTeam::Yellow)
        position_robot_flip(pos);

    return pos;
}

void ExempleStrat::reset() {
    possible_actions.clear();
    running_actions.clear();
    buildPossibleActions();
    // Wait is not a candidate action: keep it out of the pool so bestAction()
    // cannot consume it. extractAction() moves it from the pool to the
    // waitAction member, so it stays owned by the strategy; the FSM only ever
    // sees a pointer to it.
    waitAction = extractAction("Wait");
    resume(); // the strategy starts active (isActive = true)
}

/*
    Build the strategy's pool of possible actions, with their respective
    weights. The larger the weight, the more likely the action is to be chosen
    by bestAction() when it is available (cf. the score computation below).

*/
void ExempleStrat::buildPossibleActions() {

    auto addAction = [this](float weight, std::unique_ptr<VirtualAction> action) {
        std::string key = action->getNom();
        auto result = possible_actions.emplace(key, std::make_pair(weight, std::move(action)));
        if (!result.second) {
            // emplace() does not replace when the key already exists: two
            // actions with the same name would otherwise silently overwrite
            // each other.
            LOG_WARNING("ExempleStrat: action '", key.c_str(), "' already in the pool, ignored");
        }
    };

    //addAction(0.0f, std::make_unique<WaitAction>(400));
    //addAction(1.0f, std::make_unique<CalibrationAction>(robot, tableStatus));
    addAction(0.5f, std::make_unique<NavHomeAction>(tableStatus, robot, StratFinishingPos()));
    addAction(1.0f, std::make_unique<DropStoneAction>(tableStatus, robot));
    addAction(1.0f, std::make_unique<TakeStoneAction>(tableStatus, robot));
}

VirtualAction* ExempleStrat::tempAction() {
    return waitAction.get();
}

VirtualAction* ExempleStrat::bestAction() {
    if (!isActive) {
        return nullptr;
    }

    if (possible_actions.empty()) {
        LOG_WARNING("ExempleStrat: no more actions available in the pool");
        return nullptr;
    }

    std::string bestKey;
    float bestScore = -1.0f;

    for (const auto& [key, weightedAction] : possible_actions) {
        float reward;
        bool available = weightedAction.second->available(reward);
        if (!available) {
            //LOG_WARNING("ExempleStrat: cannot select action ", key.c_str());
            continue; // action not available
        }
        float score = weightedAction.first * reward;
        if (score > bestScore) {
            bestScore = score;
            bestKey = key;
        }
    }

    if (bestKey.empty()) {
        return nullptr;
    }

    LOG_GREEN_INFO("ExempleStrat: selecting action ", bestKey.c_str());

    // The action is not removed from the pool: the strategy keeps ownership,
    // the FSM only gets a pointer and can replay it while it stays available.
    return findAction(bestKey);
}