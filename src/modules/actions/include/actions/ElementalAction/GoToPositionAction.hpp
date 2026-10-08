#pragma once
#include "actions/VirtualAction.hpp"
#include "navigation/driveControl.h" // pour position_t

/*
    Action générique : déplacement vers une position donnée sur la table.
*/
class GoToPositionAction : public VirtualAction {
    public:
        GoToPositionAction(const std::string& name, position_t target, DriveControl* drive);
        ~GoToPositionAction() override = default;

        ReturnFSM_t run() override;
        bool stop() override;
        void reset() override;
        bool available(float &reward) override;
        bool fullBlock() override;
        bool mouvementBlock() override;

    protected:
        bool errorManagement();
        bool successManagement();
    private:
        position_t target;
        DriveControl* drive;
        bool moving;

        // Passe à true dès que l'action a rendu DONE ou ERROR : elle n'est
        // alors plus candidate (available() renvoie -1) jusqu'à reset().
        bool done = false;
};