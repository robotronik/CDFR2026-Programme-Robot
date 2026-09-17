#pragma once
#include "actions/VirtualAction.hpp"
#include "ElementalAction/WaitAction.hpp"
#include "ElementalAction/CalibrationAction.hpp"
#include "ElementalAction/NavHomeAction.hpp"

class ActionFSM{
    public:
        ActionFSM();
        ~ActionFSM();
        void Reset();
        bool RunFSM();

    private:
        /***** FUNCTIONS  *******/
        // Détermine et renvoie l'action la plus prioritaire à exécuter
        VirtualAction* SetBestAction();

        /************  INSTANCES DES ACTIONS ************/
        WaitAction waitAction;
        CalibrationAction calibrationAction;
        NavHomeAction navHomeAction;

        // Action actuellement en cours d'exécution
        VirtualAction* currentAction = nullptr;
};