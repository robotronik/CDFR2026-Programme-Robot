#include "actions/ElementalAction/TakeStock.hpp"
#include "defs/tableState.hpp"
#include "navigation/driveControl.h" // pour position_t
#include "constante.h"

bool TakeStock::getBestQuarry(const position_t& from, position_t& target) {
    int quarry[10] = tableState->quarryAvailable;
    if (quarry == {0,0,0,0,0,0,0,0,0,0}) {
        return false;
    }

    position_t closest = QUARRY_POSITION[0];
    double minDistance = position_distance(from, {QUARRY_POSITION[0].x, QUARRY_POSITION[0].y, QUARRY_POSITION[0].a});
    int maxstone = quarry[0];
    
    int taille = sizeof(quarry);

    for (int i = 0; i < taille;i++ ) {
        double distance = position_distance(from, {QUARRY_POSITION[i].x,QUARRY_POSITION[i].y ,QUARRY_POSITION[i].a });
        int stone = quarry[i];
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