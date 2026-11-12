#include "actions/ElementalAction/TakeStock.hpp"
#include "defs/tableState.hpp"

bool TakeStock::getBestQuarry(const position_t& from, position_t& target) {
    int quarry[] = tableState->quarryAvailable;
    if (quarry == {0,0,0,0,0,0,0,0,0,0}) {
        return false;
    }

    position_t closest = position_t QUARRY_POSITION[0];
    double minDistance = position_distance(from, {closest[0][0], closest[0][1], closest[0][2]});
    int maxstone = quarry[0];
    
    int taille = sizeof(quarry);

    for (int i = 0; i < taille;i++ ) {
        double distance = position_distance(from, {QUARRY_POSITION[i][0],QUARRY_POSITION[i][1] ,QUARRY_POSITION[i][2] });
        int stone = quarry[i];
        if (distance < minDistance) and (stone>=maxstone) {
            minDistance = distance;
            closest = QUARRY_POSITION[0];
            maxstone = stone;
        }
    }

    target.x = closest->x;
    target.y = closest->y;
    target.a = closest->a;
    return true;.
    
}