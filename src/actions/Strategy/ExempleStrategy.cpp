#include "actions/Strategy/ExempleStrat.hpp"
#include "actions/strats.hpp"
#include "navigation/driveControl.h"
#include "utils/logger.hpp"
#include "navigation/pathfind.h"
#include "actions/ElementalAction/WaitAction.hpp"
#include "actions/ElementalAction/CalibrationAction.hpp"
#include "actions/ElementalAction/NavHomeAction.hpp"
//#include "actions/ElementalAction/GoToPositionAction.hpp"
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

ExempleStrat::ExempleStrat(DriveControl* dc, TableState* ts){
    nom = "ExempleStrat";
    drive = dc;
    tableStatus = ts;
    reset();
}

void ExempleStrat::reset(){
    possible_actions.clear();
    running_actions.clear();
    buildPossibleActions();
    resume(); // la stratégie démarre active (status = true)
}

/*
    Construit le pool d'actions possibles de la stratégie, avec leur
    pondération respective. Plus la pondération est grande, plus
    l'action a de chances d'être choisie par bestAction() lorsqu'elle
    est disponible (cf. calcul du score plus bas).

    Ici on adapte la position de départ de strats.cpp (StratStartingPos)
    et on enchaîne quelques actions d'exemple.
*/
void ExempleStrat::buildPossibleActions(){
    check(tableStatus->colorTeam, tableStatus->strategy);

    //position_t objective = {0, 1000, 0}; // exemple, à adapter

    auto addAction = [this](float weight, std::unique_ptr<VirtualAction> action){
        std::string key = action->getNom();
        auto result = possible_actions.emplace(key, std::make_pair(weight, std::move(action)));
        if (!result.second){
            // emplace() ne remplace pas si la clé existe déjà : deux
            // actions avec le même nom écraseraient silencieusement l'une
            // l'autre sinon.
            LOG_WARNING("ExempleStrat: action '", key.c_str(), "' déjà présente dans le pool, ignorée");
        }
    };

    addAction(1.0f, std::make_unique<WaitAction>(400));
    addAction(1.0f, std::make_unique<CalibrationAction>(drive, tableStatus));
    addAction(1.0f, std::make_unique<NavHomeAction>(tableStatus, drive));
    //addAction(1.0f, std::make_unique<GoToPositionAction>("SecondAction", objective));
}

std::unique_ptr<VirtualAction> ExempleStrat::tempAction(){
    return extractAction("Wait");
}

std::unique_ptr<VirtualAction> ExempleStrat::bestAction(){
    if (!status){
        return nullptr;
    }

    if (possible_actions.empty()){
        LOG_WARNING("ExempleStrat: plus d'action disponible dans le pool");
        return nullptr;
    }

    std::string bestKey;
    float bestScore = -1.0f;

    for (const auto& [key, weightedAction] : possible_actions){
        float cost = weightedAction.second->available();
        if (cost < 0.0f) continue; // action non disponible

        float score = weightedAction.first * cost;
        if (score > bestScore){
            bestScore = score;
            bestKey = key;
        }
    }

    if (bestKey.empty()){
        return nullptr;
    }

    LOG_GREEN_INFO("ExempleStrat: sélection de l'action ", bestKey.c_str());

    // extractAction() fait exactement le find + move + erase qu'on
    // faisait "à la main" avec l'index dans la version vector.
    return extractAction(bestKey);
}