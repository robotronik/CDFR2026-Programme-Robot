#pragma once
#include "actions/VirtualStrategy.hpp"
#include "actions/VirtualAction.hpp"

#include "navigation/driveControl.h"
#include "defs/tableState.hpp"

/*
    ExempleStrat
    ------------
    Exemple de "stratégie" au sens de VirtualStrategy : contrairement à
    une action unique à états, une VirtualStrategy expose un ensemble
    d'actions possibles, chacune pondérée, et choisit à chaque appel de
    bestAction() la plus intéressante/prioritaire.
    
    - chaque action candidate sait dire si elle est jouable maintenant
      via available() (cf. VirtualAction::available()),
    - la pondération (poids fixé à la construction) permet de hiérarchiser
      des actions qui seraient toutes disponibles en même temps.
*/
class ExempleStrat : public VirtualStrategy {
public:
    ExempleStrat(DriveControl* dc, TableState* ts);
    ~ExempleStrat() override = default;

    /*
        Sélectionne, parmi possible_actions, l'action dont le score
        (pondération * action->available()) est le plus élevé parmi
        celles disponibles (available() >= 0), et la retourne en
        transférant sa propriété à l'appelant.

        Retourne nullptr si la stratégie est arrêtée (stop()) ou si
        aucune action du pool n'est disponible.
    */
    std::unique_ptr<VirtualAction> bestAction() override;
    std::unique_ptr<VirtualAction> tempAction() override;

    /* Ré-remplit le pool d'actions possibles (à appeler par exemple au
       Reset() du FSM, ou quand la stratégie a écoulé toutes ses actions
       et doit être rejouée depuis le début). */
    void reset();

private:
    // (Re)construit le pool possible_actions pour la stratégie courante
    void buildPossibleActions();
    DriveControl* drive;
    TableState* tableStatus;
};