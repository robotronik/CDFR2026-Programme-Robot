#pragma once
#include "actions/VirtualAction.hpp"
#include "navigation/driveControl.h" // pour position_t

class TableState;

/*
    Action générique : déplacement vers une position donnée sur la table.
    Deux modes :
      - cible fixe, fournie à la construction (position_t) ;
      - cible dynamique, choisie au démarrage de l'action comme l'élément de
        jeu le plus proche du robot parmi ceux mémorisés dans TableState
        (cf. GoToPositionAction::getClosestElement()).
*/
class GoToPositionAction : public VirtualAction {
    public:
        GoToPositionAction(const std::string& name, position_t target, DriveControl* drive);
        GoToPositionAction(const std::string& name, TableState* tableState, DriveControl* drive);
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
        // Désigne, parmi les éléments de jeu mémorisés dans tableState, le plus
        // proche de `from`. Renseigne `target` et renvoie false si aucun
        // élément n'est connu.
        bool getClosestElement(const position_t& from, position_t& target) const;
        // Résout la cible dynamique à partir de tableState. Sans effet sur une
        // cible fixe. Renvoie false si aucun élément n'est connu.
        bool resolveTarget();

        position_t target;
        DriveControl* drive;
        // Source des cibles dynamiques (non possédée). Nul pour une cible fixe.
        TableState* tableState = nullptr;
        // Vrai tant que la cible dynamique doit être (re)calculée.
        bool needsTarget = false;
        bool moving;
        int value;
        // Passe à true dès que l'action a rendu DONE ou ERROR : elle n'est
        // alors plus candidate (available() renvoie -1) jusqu'à reset().
        bool done = false;
};