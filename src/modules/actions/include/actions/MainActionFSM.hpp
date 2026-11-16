#pragma once
#include "actions/VirtualAction.hpp"
#include "actions/VirtualStrategy.hpp"
#include "db/ActionDurationDB.hpp"
#include "defs/tableState.hpp"

class RobotControl;

class ActionFSM{
    public:
        ActionFSM(RobotControl* robot, TableState* tableState);
        explicit ActionFSM(VirtualStrategy* strategy, RobotControl* robot, TableState* tableState);
        ~ActionFSM();
        void Reset();
        bool RunFSM();

        bool endlessMod = false;
        // Permet de brancher/changer la stratégie utilisée par le FSM.
        // N'importe quelle classe dérivant de VirtualStrategy (ExempleStrat,
        // ou toute autre stratégie de match) peut être utilisée ici.
        void setStrategy(VirtualStrategy* strategy);
        VirtualStrategy* getStrategy(){ return currentStrategy; }

    private:
        RobotControl* robotControl = nullptr;
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
        // Enregistre, pour chaque action menée à son terme (ReturnFSM::Done),
        // le temps qu'elle a réellement mis à s'exécuter.
        ActionDurationDB durationDB{ACTION_DB_PATH};
        // Instant (_millis()) du début de l'exécution de currentAction.
        unsigned long actionStartTime = 0;
};