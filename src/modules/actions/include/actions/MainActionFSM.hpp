#pragma once
#include "actions/VirtualAction.hpp"
#include "actions/VirtualStrategy.hpp"
//#include "db/ActionDurationDB.hpp"
#include "defs/tableState.hpp"

class Robot;

class ActionFSM{
    public:
        ActionFSM(Robot* robot, TableState* tableState);
        explicit ActionFSM(VirtualStrategy* strategy, Robot* robot, TableState* tableState);
        ~ActionFSM();
        void Reset();
        bool RunFSM();

        bool isEndlessMode = false;
        // Permet de brancher/changer la stratégie utilisée par le FSM.
        // N'importe quelle classe dérivant de VirtualStrategy (ExempleStrat,
        // ou toute autre stratégie de match) peut être utilisée ici.
        VirtualStrategy* getStrategy(){ return currentStrategy; }

    private:
        Robot* robotControl = nullptr;
        TableState* tableState = nullptr;
        /***** FUNCTIONS  *******/
        // Pull the strategy currently selected on the robot; drop the running
        // action if it changed.
        void syncStrategy();
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
        //ActionDurationDB durationDB{ACTION_DB_PATH};
        // Instant (_millis()) du début de l'exécution de currentAction.
        unsigned long actionStartTime = 0;
};