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
        float available() override;
        bool fullBlock() override;
        bool mouvementBlock() override;

    protected:
        bool errorManagement();
        bool successManagement();
    private:
        position_t target;
        DriveControl* drive;
        bool moving;
};