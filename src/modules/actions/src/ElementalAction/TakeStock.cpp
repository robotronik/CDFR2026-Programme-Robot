#include "actions/ElementalAction/TakeStock.hpp"
#include "defs/tableState.hpp"
#include "navigation/driveControl.h" // pour position_t
#include "defs/constante.h"
#include "GoToPositionAction.cpp"
#include "navigation/navigation.h" // pour nav_return_t & position_t

TakeStock::TakeStock(TableState* tableStatus, DriveControl* drive)
    : tableState(tableStatus)
{
    state = MOVE1;
}

bool TakeStock::getBestQuarry(const position_t& from, position_t& target) {
    if (tableState->remainQuarry()) {
        return false;
    }

    position_t closest = QUARRY_POSITION[0];
    double minDistance = position_distance(from, {QUARRY_POSITION[0].x, QUARRY_POSITION[0].y, QUARRY_POSITION[0].a});
    //TODO implement A* (pathfind a la place de position_distance)
    int maxstone = tableState->isQuarryAvailable(0) ;
    
    int taille = sizeof(tableState->quarry);

    for (int i = 0; i < taille;i++ ) {
        double distance = position_distance(from, {QUARRY_POSITION[i].x,QUARRY_POSITION[i].y ,QUARRY_POSITION[i].a });
        int stone = tableState->isQuarryAvailable(i);
        if (distance < minDistance  && stone>=maxstone) {
            minDistance = distance;
            closest = QUARRY_POSITION[i];
            maxstone = stone;
        }
    }

    target.x = closest.x;
    target.y = closest.y;
    target.a = closest.a;
    return true;
    
}

position_t TakeStock::BestPositionToTake(const position_t& from, position_t& target) {
    position_t diffpos = {target.x, target.y, from.a};
    //pour les quarry pour prendre autre part il faut faire autrement
    if (target.y == 950) {
        target.y = 800;
    }
    else if (target.y == -950)
    {
        target.y = -800;
    }
    else {
        if (from.x >= 0) {
            target.x = 150;
        }
        else if (from.x < 0)
        {
            target.x = -150;
        }
        
    }
    // TODO il reste target.a a modifié mais il depend de la face voulue et diffpos a, et prendre en compte le decalage des pompes par rapport au centre du robot

    return diffpos;

}

bool TakeStock::BestTake(const position_t& from, position_t& target) {
    return getBestQuarry(from, target);//TODO ajouter le vole des bloques ou la recuperation des bloques hors carriere
}

ReturnFSM_t TakeStock::run(){
    getBestQuarry(drive->getPosition(), target);
    const position_t etape = BestPositionToTake(drive->getPosition(), target);

    switch (MOVE) {
        case MOVE1:
            nav_return_t res1 = navigation.goTo(target, true, false, false);
            switch (res1) {
                case NAV_PAUSED:
                    LOG_DEBUG("TakeStock: nav is paused");
                    moving = false;
                    break;
                case NAV_ERROR:
                    errorManagement();
                    return FSM_RETURN_ERROR;
                case NAV_DONE:
                    return MOVE1;
                case NAV_IN_PROCESS:
                    moving = true;
                    break;
            }
        case MOVE2:
            //allume les pompes 
            nav_return_t res2 = navigation.goTo(etape, true, true, true);//TODO fast mode
            switch (res2) {
                case NAV_PAUSED:
                    LOG_DEBUG("TakeStock: nav is paused");
                    moving = false;
                    break;
                case NAV_ERROR:
                    errorManagement();
                    return FSM_RETURN_ERROR;
                case NAV_DONE:
                    successManagement();
                    return FSM_RETURN_DONE;
                case NAV_IN_PROCESS:
                    moving = true;
                    break;

        }
        
        
    }
    return FSM_RETURN_WORKING;

}


bool TakeStock::stop(){
    if(moving){
        drive->stopMotion();
        moving = false;
    }
    return true;
}

bool TakeStock::errorManagement(){
    LOG_ERROR("TakeStock: erreur de navigation vers ", nom.c_str());
    reset();
    return true;
}

bool TakeStock::successManagement(){
    LOG_GREEN_INFO("TakeStock: arrivé à ", nom.c_str());
    moving = false;
    done = true;
    return true;
}

void TakeStock::reset(){
    stop();
    done = false;                  // l'action redevient candidate
    needsTarget = (tableState != nullptr); // re-cible l'élément le plus proche
}

bool TakeStock::available(float &reward){
    // Déplacement déjà terminé (succès ou échec) : plus candidate tant
    // qu'elle n'a pas été réarmée par reset().
    if (done) return false;
    // Cible dynamique : sans élément connu, l'action n'est pas jouable.
    if (!resolveTarget()) return false;
    double path_length_mm;
    position_t path[100]; // Assuming a maximum path length
    if(!pathfind(drive->getPosition(), target, path, path_length_mm)){
        return false;
    }
    reward = value/path_length_mm;
    return true;
}

bool TakeStock::resolveTarget(){
    if (!needsTarget) {
        return true; // cible fixe, ou déjà résolue pour cette exécution
    }
    if (!BestTake(drive->getPosition(), target)) {
        return false; // aucun élément de jeu connu
    }
    needsTarget = false;
    return true;
}

bool TakeStock::fullBlock(){
    return false;
}

bool TakeStock::mouvementBlock(){
    return moving;
}