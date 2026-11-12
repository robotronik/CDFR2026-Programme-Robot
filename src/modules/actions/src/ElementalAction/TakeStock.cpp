#include "actions/ElementalAction/TakeStock.hpp"
#include "defs/tableState.hpp"
#include "navigation/driveControl.h" // pour position_t
#include "constante.h"
#include "src/modules/actions/src/ElementalAction/GoToPositionAction.cpp"

bool TakeStock::getBestQuarry(const position_t& from, position_t& target) {
    quarry_t quarry[10] = tableState->quarryAvailable;
    if (tableState->remainQuarry()) {
        return false;
    }

    position_t closest = QUARRY_POSITION[0];
    double minDistance = position_distance(from, {QUARRY_POSITION[0].x, QUARRY_POSITION[0].y, QUARRY_POSITION[0].a});
    int maxstone = quarry[0].nb ;
    
    int taille = sizeof(quarry);

    for (int i = 0; i < taille;i++ ) {
        double distance = position_distance(from, {QUARRY_POSITION[i].x,QUARRY_POSITION[i].y ,QUARRY_POSITION[i].a });
        int stone = quarry[i].nb;
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
        target.y == -800;
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
    // TODO il reste target.a a modifié mais il depend de la face voulue et diffpos a

    return diffpos

}

ReturnFSM_t run(){

//se deplace a target

//allume les pompes

//avance pour prendre les blocs
}