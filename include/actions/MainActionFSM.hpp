#pragma once
#include <memory>
#include "actions/VirtualAction.hpp"
#include "actions/VirtualStrategy.hpp"
#include "db/ActionDurationDB.hpp"
#include "defs/tableState.hpp"
#include "navigation/driveControl.h"

class ActionFSM{
    public:
        ActionFSM(DriveControl* drive, TableState* tableState);
        explicit ActionFSM(VirtualStrategy* strategy, DriveControl* drive, TableState* tableState);
        ~ActionFSM();
        void Reset();
        bool RunFSM();

        // Permet de brancher/changer la stratégie utilisée par le FSM.
        // N'importe quelle classe dérivant de VirtualStrategy (ExempleStrat,
        // ou toute autre stratégie de match) peut être utilisée ici.
        void setStrategy(VirtualStrategy* strategy);

    private:
        DriveControl* driveControl = nullptr;
        TableState* tableState = nullptr;
        /***** FUNCTIONS  *******/
        // Détermine et renvoie l'action la plus prioritaire à exécuter
        VirtualAction* SetBestAction();

        /************  STRATÉGIE COURANTE ************/
        // La stratégie n'appartient pas au FSM (juste référencée) : elle
        // peut être partagée/recréée ailleurs (ex: choisie selon le
        // switch de couleur/stratégie sur le robot).
        VirtualStrategy* currentStrategy = nullptr;

        // Action actuellement en cours d'exécution. Elle appartient à
        // currentStrategy (bestAction() ou tempAction()) : le FSM ne la
        // possède pas et ne doit jamais la libérer.
        VirtualAction* currentAction = nullptr;

        /************  BASE DES DURÉES D'ACTION ************/
        // Enregistre, pour chaque action menée à son terme (FSM_RETURN_DONE),
        // le temps qu'elle a réellement mis à s'exécuter.
        ActionDurationDB durationDB{ACTION_DB_PATH};
        // Instant (_millis()) du début de l'exécution de currentAction.
        unsigned long actionStartTime = 0;
};