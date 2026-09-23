#include "actions/Strategy/ExempleStrat.hpp"
#include "actions/strats.hpp"
#include "utils/logger.hpp"
#include "main.hpp" // pour tableStatus, _millis()
#include "navigation/pathfind.h"
#include "actions/ElementalAction/WaitAction.hpp"
#include "actions/ElementalAction/CalibrationAction.hpp"
#include "actions/ElementalAction/NavHomeAction.hpp"
#include "actions/ElementalAction/GoToPositionAction.hpp"
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

ExempleStrat::ExempleStrat(){
    nom = "ExempleStrat";
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
    check(tableStatus.colorTeam, tableStatus.strategy);

    position_t objective = {0, 1000, 0}; // exemple, à adapter

    possible_actions.emplace_back(
        1.0f, // pondération
        std::make_unique<WaitAction>(400)
    );
    possible_actions.emplace_back(
        1.0f,
        std::make_unique<CalibrationAction>()
    );
    possible_actions.emplace_back(
        1.0f,
        std::make_unique<NavHomeAction>()
    );
    possible_actions.emplace_back(
        1.0f,
        std::make_unique<GoToPositionAction>("SecondAction", objective)
    );
}

std::unique_ptr<VirtualAction> ExempleStrat::bestAction(){
    if (!status){
        // Stratégie arrêtée (stop()) : on ne propose plus rien.
        return nullptr;
    }

    if (possible_actions.empty()){
        LOG_WARNING("ExempleStrat: plus d'action disponible dans le pool");
        return nullptr;
    }

    // Recherche, parmi possible_actions, celle avec le meilleur score.
    // score = pondération * available() ; available() < 0 => action
    // écartée (non jouable dans le contexte actuel).
    int bestIndex = -1;
    float bestScore = -1.0f;

    for (size_t i = 0; i < possible_actions.size(); ++i){
        float cost = possible_actions[i].second->available();
        if (cost < 0.0f) continue; // action non disponible

        float score = possible_actions[i].first * cost;
        if (score > bestScore){
            bestScore = score;
            bestIndex = static_cast<int>(i);
        }
    }

    if (bestIndex < 0){
        // Aucune action du pool n'est jouable actuellement
        return nullptr;
    }

    LOG_GREEN_INFO("ExempleStrat: sélection de l'action ",
                    possible_actions[bestIndex].second->getNom().c_str());

    // On retire l'action choisie du pool : elle est dispatchée une
    // seule fois (la stratégie ne la reproposera pas). Si vous voulez
    // qu'une action puisse revenir dans le pool, ré-ajoutez-la ici
    // après son exécution (ex: depuis MainActionFSM une fois FSM_RETURN_DONE reçu).
    std::unique_ptr<VirtualAction> chosen = std::move(possible_actions[bestIndex].second);
    possible_actions.erase(possible_actions.begin() + bestIndex);

    return chosen;
}