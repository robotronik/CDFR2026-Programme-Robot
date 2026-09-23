#pragma once
#include <memory>
#include "actions/VirtualAction.hpp"
#include "actions/VirtualStrategy.hpp"
#include "ElementalAction/WaitAction.hpp"
#include "ElementalAction/CalibrationAction.hpp"
#include "ElementalAction/NavHomeAction.hpp"

class ActionFSM{
    public:
        ActionFSM();
        explicit ActionFSM(VirtualStrategy* strategy);
        ~ActionFSM();
        void Reset();
        bool RunFSM();

        // Permet de brancher/changer la stratégie utilisée par le FSM.
        // N'importe quelle classe dérivant de VirtualStrategy (ExempleStrat,
        // ou toute autre stratégie de match) peut être utilisée ici.
        void setStrategy(VirtualStrategy* strategy);

    private:
        /***** FUNCTIONS  *******/
        // Détermine et renvoie l'action la plus prioritaire à exécuter
        VirtualAction* SetBestAction();

        /************  INSTANCES DES ACTIONS "SYSTÈME" ************/
        // Ces actions sont toujours disponibles, indépendamment de la
        // stratégie en cours, et restent prioritaires sur elle.
        WaitAction waitAction = WaitAction(500);
        CalibrationAction calibrationAction;
        NavHomeAction navHomeAction;

        /************  STRATÉGIE COURANTE ************/
        // La stratégie n'appartient pas au FSM (juste référencée) : elle
        // peut être partagée/recréée ailleurs (ex: choisie selon le
        // switch de couleur/stratégie sur le robot).
        VirtualStrategy* currentStrategy = nullptr;

        // Action rendue par currentStrategy->bestAction() : le FSM en
        // récupère la propriété (unique_ptr) tant qu'elle est exécutée.
        std::unique_ptr<VirtualAction> strategyAction;

        // Action actuellement en cours d'exécution (pointe soit sur une
        // action système ci-dessus, soit sur strategyAction.get()).
        VirtualAction* currentAction = nullptr;
};