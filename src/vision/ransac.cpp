#include "vision/ransac.hpp"

/*
block_t startPosition(const std::vector<const block_t*>& choosen, const std::vector<block_t>& points, const block_t& robotPos){

}
*/

bool findGroupRANSAC2D(
    const std::vector<block_t>& points,
    std::vector<block_t>& bestGroup,
    size_t max_blocks,
    float lineTol,
    float spacing,
    float spacingTol,
    float angleTol // tolérance orientation
) {
    block_t robotPos = bestGroup.back();
    bestGroup.clear();
    
    for (size_t i = 0; i < points.size(); i++) {
        for(size_t j = i + 1; j < points.size(); j++){
            LOG_EXTENDED_DEBUG("Ransac: Testing line through points (", points[i].x, ", ", points[i].y, ") and (", points[j].x, ", ", points[j].y, ")");
            float dx = points[j].x - points[i].x;
            float dy = points[j].y - points[i].y;

            float norm = std::sqrt(dx*dx + dy*dy);
            if (norm < 1e-6f) continue;

            Line line;
            line.x = points[i].x;
            line.y = points[i].y;
            line.dx = dx / norm;
            line.dy = dy / norm;

            // angle de la ligne (en degrés)
            float lineAngle = std::atan2(line.dy, line.dx) * 180.0f / M_PI;

            // 2. collecter inliers (géométrie + orientation)
            std::vector<std::pair<float, const block_t*>> inliers;
            inliers.reserve(points.size());

            for (const auto& p : points) {

                // filtre orientation
                float diff = angleDiff(p.a, lineAngle);

                // make it invariant to 180° flip
                diff = std::min(diff, 180.0f - diff);
                if (diff > angleTol){
                    LOG_EXTENDED_DEBUG("Ransac: Ignoring point (", p.x, ", ", p.y, ") with angle ", p.a, " due to orientation difference ", diff);
                    continue;
                }

                float d = pointLineDistance(p, line);
                if (d < lineTol) {
                    float proj = project(p, line);
                    inliers.emplace_back(proj, &p);
                    LOG_EXTENDED_DEBUG("Ransac: Point (", p.x, ", ", p.y, ") with angle ", p.a, " is an inlier (distance ", d, ", orientation diff ", diff, ")");
                }else{
                    LOG_EXTENDED_DEBUG("Ransac: Ignoring point (", p.x, ", ", p.y, ") with angle ", p.a, " due to distance ", d);
                }
            }

            if (inliers.size() < max_blocks) continue;

            LOG_EXTENDED_DEBUG("Ransac: Found ", inliers.size(), " inliers for line through points (", points[i].x, ", ", points[i].y, ") and (", points[j].x, ", ", points[j].y, ") with angle ", lineAngle);
            std::sort(inliers.begin(), inliers.end(),
                    [](const std::pair<float, const block_t*>& a, const std::pair<float, const block_t*>& b) {
                        return a.first > b.first;
                    });
            LOG_EXTENDED_DEBUG("Ransac: Inliers sorted by projection: ", inliers.size(), " points");

            // vérification de l'espacement
            for (size_t k = 0; k + max_blocks - 1 < inliers.size(); k++) {
                bool status = true;

                for(size_t spaces = 0; spaces < max_blocks-1; spaces ++ ){
                    float d1 = inliers[k + spaces + 1].first - inliers[k + spaces].first;
                    if(abs(abs(d1) - spacing) < spacingTol){
                        LOG_EXTENDED_DEBUG("Ransac: Inliers OK at positions ", inliers[k + spaces].second->x, ", ", inliers[k + spaces].second->y, " and ", inliers[k + spaces + 1].second->x, ", ", inliers[k + spaces + 1].second->y, " have spacing ", d1, " which is within the tolerance");
                        continue;
                    }else{
                        LOG_EXTENDED_DEBUG("Ransac: Inliers NOT OK at positions ", inliers[k + spaces].second->x, ", ", inliers[k + spaces].second->y, " and ", inliers[k + spaces + 1].second->x, ", ", inliers[k + spaces + 1].second->y, " have spacing ", d1, " which is outside the tolerance");
                        status = false;
                        break;
                    }
                }
                if (!status)
                {
                    continue;
                }
                
                if(blockInFrontInterface(std::vector<std::pair<float, const block_t*>>(inliers.begin() + k, inliers.begin() + k + max_blocks), points)){
                    LOG_EXTENDED_DEBUG("Ransac: Group of inliers at positions ", inliers[k].second->x, ", ", inliers[k].second->y, " to ", inliers[k + max_blocks - 1].second->x, ", ", inliers[k + max_blocks - 1].second->y, " is in front of the robot, rejecting this group");
                    continue;
                }

                for (size_t idx = 0; idx < max_blocks; ++idx) {
                    bestGroup.push_back(*inliers[k + idx].second);
                    LOG_EXTENDED_DEBUG("Ransac: Adding point (", inliers[k + idx].second->x, ", ", inliers[k + idx].second->y, ") with angle ", inliers[k + idx].second->a, " to best group");
                }
                
                std::sort(bestGroup.begin(), bestGroup.end(),
                [](const block_t& a, const block_t& b) {
                    return a.y > b.y;
                });

                if(bestGroup.size() !=2){
                    robotPos = transformRobotCoordToTableCoord(bestGroup[1], robotPos, false);
                }else{
                    robotPos = transformRobotCoordToTableCoord(bestGroup[0], robotPos, false);
                }
                if(isRobotInWall(robotPos)){
                    continue;
                }
                
                float ux, uy;
                if (bestGroup.size() >= 2) {
                    ux = bestGroup.back().x - bestGroup.front().x;
                    uy = bestGroup.back().y - bestGroup.front().y;
                }
                float len = std::sqrt(ux*ux + uy*uy);
                if (len > 1e-6f) {
                    ux /= len;
                    uy /= len;
                } else {
                    float a_rad = bestGroup.front().a * M_PI / 180.0f;
                    ux = std::cos(a_rad);
                    uy = std::sin(a_rad);
                }
                float lineAngle = std::atan2(uy, ux) * 180.0f / M_PI;
                float pa = lineAngle + 90.0f;
                
                for (size_t i = 0; i < bestGroup.size(); i++)
                {
                    bestGroup[i].a = pa;
                    LOG_EXTENDED_DEBUG("Ransac: Setting angle of point (", bestGroup[i].x, ", ", bestGroup[i].y, ") to line angle ", lineAngle);
                }
                
                return true;
            }
        }
        
    }
    LOG_WARNING("Ransac take: No valid group of ", max_blocks, " inliers found");
    return false;
}

bool findGroupStealRANSAC2D(
    const std::vector<block_t>& points,
    std::vector<block_t>& bestGroup,
    size_t max_blocks,
    float lineTol,
    float spacing,
    float spacingTol,
    float angleTol
){
    block_t robotPos = bestGroup.back();
    bestGroup.clear();

    for (size_t i = 0; i < points.size(); i++) {
        for(size_t j = i + 1; j < points.size(); j++){
            float dx = points[j].x - points[i].x;
            float dy = points[j].y - points[i].y;

            float norm = std::sqrt(dx*dx + dy*dy);
            if (norm < 1e-6f) continue;

            Line line;
            line.x = points[i].x;
            line.y = points[i].y;
            line.dx = dx / norm;
            line.dy = dy / norm;

            // angle de la ligne (en degrés)
            float lineAngle = std::atan2(line.dy, line.dx) * 180.0f / M_PI;

            // 2. collecter inliers (géométrie + orientation)
            std::vector<std::tuple<float, const block_t*, bool>> inliers;
            std::vector<std::tuple<float, const block_t*, bool>> sol_temp;

            inliers.reserve(points.size());

            for (size_t k =0; k < points.size(); k++) {
                const auto& p = points[k];
                float d = pointLineDistance(p, line);
                if (d < lineTol) {
                    float proj = project(p, line);
                    inliers.emplace_back(proj, &p, k == i || k == j );
                }
            }
            
            std::sort(inliers.begin(), inliers.end(),
                    [](const std::tuple<float, const block_t*, bool>& a, const std::tuple<float, const block_t*, bool>& b) {
                        return std::get<0>(a) > std::get<0>(b);
                    });
            
            if (inliers.size() < max_blocks) continue; // pas assez de blocs on skip

            // --- 1. Retrouver les indices des blocs de base ---
            int idx_seed_min = -1;
            int idx_seed_max = -1;
            for (int m = 0; m < static_cast<int>(inliers.size()); ++m) {
                if (std::get<2>(inliers[m])) { // Le booléen indique que c'est le bloc i ou j
                    if (idx_seed_min == -1) {
                        idx_seed_min = m;
                    } else {
                        idx_seed_max = m;
                        break; // On a trouvé les deux, on s'arrête
                    }
                }
            }

            // --- 2. Complétion de la solution ---
            // statu de la solution true si valable
            bool status = true;

            // Si trop de blocs entre les 2 blocs choisis, solution invalide
            if (idx_seed_max - idx_seed_min > 3) {
                LOG_EXTENDED_DEBUG("Ransac steal: Too many blocks between seed blocks at indices ", idx_seed_min, " and ", idx_seed_max, ", rejecting this group");
                continue;
            }
            sol_temp.emplace_back(inliers[idx_seed_min]);
            // On ajoute le bloc idx_seed_max choisis ET ceux qui sont entre eux (s'il y en a)
            for (int m = idx_seed_min + 1; m <= idx_seed_max; ++m) {
                if( acceptableAngle(std::get<1>(inliers[m]), std::get<1>(sol_temp.back()), angleTol )){
                    sol_temp.emplace_back(inliers[m]);
                }else{
                    status = false;
                    LOG_EXTENDED_DEBUG("Ransac steal: Inlier at index ", m, " is not acceptable, rejecting this group");
                    LOG_EXTENDED_DEBUG("Ransac steal: Inlier at index ", m, " has angle ", std::get<1>(inliers[m])->a, " which is not within the tolerance of the previous inlier with angle ", std::get<1>(sol_temp.back())->a);
                    break;
                }

            }

            // On complète avec les inliers d'indices supérieurs à idx_seed_max (vers la droite)
            int current_right = idx_seed_max + 1;
            while (status && sol_temp.size() < max_blocks && current_right < static_cast<int>(inliers.size())) {
                if( acceptableAngle(std::get<1>(inliers[current_right]), std::get<1>(sol_temp.back()), angleTol )){
                    sol_temp.emplace_back(inliers[current_right]);
                    current_right++;
                }else{
                    status = false;
                    LOG_EXTENDED_DEBUG("Ransac steal: Inlier at index ", current_right, " is not acceptable, rejecting this group");
                    LOG_EXTENDED_DEBUG("Ransac steal: Inlier at index ", current_right, " has angle ", std::get<1>(inliers[current_right])->a, " which is not within the tolerance of the previous inlier with angle ", std::get<1>(sol_temp.back())->a);
                    break;
                }
            }
            
            // On complète avec les inliers d'indices inférieurs à idx_seed_min (vers la gauche, limite 0)
            int current_left = idx_seed_min - 1;
            while (status && sol_temp.size() < max_blocks && current_left >= 0) {
                if( acceptableAngle(std::get<1>(inliers[current_left]), std::get<1>(sol_temp.front()), angleTol )){
                    sol_temp.insert(sol_temp.begin(), inliers[current_left]);
                    current_left--;
                }else{
                    status = false;
                    LOG_EXTENDED_DEBUG("Ransac steal: Inlier at index ", current_left, " is not acceptable, rejecting this group");
                    LOG_EXTENDED_DEBUG("Ransac steal: Inlier at index ", current_left, " has angle ", std::get<1>(inliers[current_left])->a, " which is not within the tolerance of the previous inlier with angle ", std::get<1>(sol_temp.front())->a);
                    break;
                }
            }
            if(!status) continue;

            if(blockInFrontInterface(sol_temp, points)){
                LOG_EXTENDED_DEBUG("Ransac steal: Group of inliers at positions ", std::get<1>(sol_temp.front())->x, ", ", std::get<1>(sol_temp.front())->y, " to ", std::get<1>(sol_temp.back())->x, ", ", std::get<1>(sol_temp.back())->y, " is in front of the robot, rejecting this group");
                continue;
                //TODO gérer et trouver une solution en passant par l'autre coté?
            }

            block_t pouss = interfacePlacePoussoir(sol_temp, points, robotPos);
            bestGroup.clear();
            if(pouss.color){
                bestGroup.push_back(pouss);
                double distance = std::hypot(pouss.x - std::get<1>(sol_temp[0])->x, pouss.y - std::get<1>(sol_temp[0])->y);
                float ux, uy;
                if (sol_temp.size() >= 2) {
                    ux = std::get<1>(sol_temp.back())->x - std::get<1>(sol_temp.front())->x;
                    uy = std::get<1>(sol_temp.back())->y - std::get<1>(sol_temp.front())->y;
                }
                float len = std::sqrt(ux*ux + uy*uy);
                if (len > 1e-6f) {
                    ux /= len;
                    uy /= len;
                } else {
                    float a_rad = std::get<1>(sol_temp.front())->a * M_PI / 180.0f;
                    ux = std::cos(a_rad);
                    uy = std::sin(a_rad);
                }

                block_t info = { .x = distance, .y = 0, .a = std::atan2(uy, ux) * 180.0f / M_PI + 90.0f, .color = pouss.color };
                bestGroup.push_back(info);
                return true;
            }else{
                LOG_DEBUG("pas de poussoir");
                return false;
            }
            
            
        }
    }
    LOG_WARNING("Ransac steal: No valid group of ", max_blocks, " inliers found");
    return false;

}