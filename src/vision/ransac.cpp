#include "vision/ransac.hpp"

// normalisation angle [-180, 180]
inline float angleDiff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    return std::abs(d);
}

// distance point-droite 2D
inline float pointLineDistance(const block_t& p, const Line& l) {
    float vx = p.x - l.x;
    float vy = p.y - l.y;

    float cross = std::abs(vx * l.dy - vy * l.dx);
    return cross; // car dir normalisée
}

// projection scalaire
inline float project(const block_t& p, const Line& l) {
    return (p.x - l.x) * l.dx +
           (p.y - l.y) * l.dy;
}

bool acceptableAngle(const block_t* b1, const block_t* b2, float angleTol){
    float dx = b2->x - b1->x;
    float dy = b2->y - b1->y;

    float norm = std::sqrt(dx*dx + dy*dy);
    if (norm < 1e-6f) return false;

    Line line;
    line.x = b1->x;
    line.y = b1->y;
    line.dx = dx / norm;
    line.dy = dy / norm;

    // angle de la ligne (en degrés)
    float lineAngle = std::atan2(line.dy, line.dx) * 180.0f / M_PI;
    return angleDiff(b1->a, lineAngle) < angleTol;
}

bool findGroupRANSAC2D(
    const std::vector<block_t>& points,
    std::vector<block_t>& bestGroup,
    size_t max_blocks,
    float lineTol,
    float spacing,
    float spacingTol,
    float angleTol // tolérance orientation
) {
    for (int i = 0; i < points.size(); i++) {
        for(int j = i + 1; j < points.size(); i++){
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
                    //LOG_EXTENDED_DEBUG("Ransac: Ignoring point (", p.x, ", ", p.y, ") with angle ", p.a, " due to orientation difference ", diff);
                    continue;
                }

                float d = pointLineDistance(p, line);
                if (d < lineTol) {
                    float proj = project(p, line);
                    inliers.emplace_back(proj, &p);
                    //LOG_EXTENDED_DEBUG("Ransac: Point (", p.x, ", ", p.y, ") with angle ", p.a, " is an inlier (distance ", d, ", orientation diff ", diff, ")");
                }else{
                    //LOG_EXTENDED_DEBUG("Ransac: Ignoring point (", p.x, ", ", p.y, ") with angle ", p.a, " due to distance ", d);
                }
            }

            if (inliers.size() < max_blocks) continue;

            //LOG_EXTENDED_DEBUG("Ransac: Found ", inliers.size(), " inliers for line through points (", points[i].x, ", ", points[i].y, ") and (", points[j].x, ", ", points[j].y, ") with angle ", lineAngle);
            std::sort(inliers.begin(), inliers.end(),
                    [](const std::pair<float, const block_t*>& a, const std::pair<float, const block_t*>& b) {
                        return a.first > b.first;
                    });
            //LOG_EXTENDED_DEBUG("Ransac: Inliers sorted by projection: ", inliers.size(), " points");

            // vérification de l'espacement
            for (size_t k = 0; k + max_blocks - 1 < inliers.size(); k++) {
                bool status = true;

                for(size_t spaces = 0; spaces < max_blocks-1; spaces ++ ){
                    float d1 = inliers[k + spaces + 1].first - inliers[k + spaces].first;
                    if(abs(abs(d1) - spacing) < spacingTol){
                        //LOG_EXTENDED_DEBUG("Ransac: Inliers OK at positions ", inliers[k + spaces].second->x, ", ", inliers[k + spaces].second->y, " and ", inliers[k + spaces + 1].second->x, ", ", inliers[k + spaces + 1].second->y, " have spacing ", d1, " which is within the tolerance");
                        continue;
                    }else{
                        //LOG_EXTENDED_DEBUG("Ransac: Inliers NOT OK at positions ", inliers[k + spaces].second->x, ", ", inliers[k + spaces].second->y, " and ", inliers[k + spaces + 1].second->x, ", ", inliers[k + spaces + 1].second->y, " have spacing ", d1, " which is outside the tolerance");
                        status = false;
                        break;
                    }
                }

                if (status) {
                    bestGroup.clear();
                    for (size_t idx = 0; idx < max_blocks; ++idx) {
                        bestGroup.push_back(*inliers[k + idx].second);
                        //LOG_EXTENDED_DEBUG("Ransac: Adding point (", inliers[k + idx].second->x, ", ", inliers[k + idx].second->y, ") with angle ", inliers[k + idx].second->a, " to best group");
                    }
                    std::sort(bestGroup.begin(), bestGroup.end(),
                    [](const block_t& a, const block_t& b) {
                        return a.y > b.y;
                    });
                    return true; // early exit if solution found
                }
            }
        }
        
    }

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
                continue;
            }
            sol_temp.emplace_back(inliers[idx_seed_min]);
            // On ajoute le bloc idx_seed_max choisis ET ceux qui sont entre eux (s'il y en a)
            for (int m = idx_seed_min + 1; m <= idx_seed_max; ++m) {
                if( acceptableAngle(std::get<1>(inliers[m]), std::get<1>(sol_temp.back()), angleTol )){
                    sol_temp.emplace_back(inliers[m]);
                }else{
                    status = false;
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
                    break;
                }
            }
            if(!status) continue;

            for (const auto& p : points) {

                // filtre orientation
                float diff = angleDiff(p.a, lineAngle);

                // make it invariant to 180° flip
                diff = std::min(diff, 180.0f - diff);
                if (diff > angleTol){
                    //LOG_EXTENDED_DEBUG("Ransac: Ignoring point (", p.x, ", ", p.y, ") with angle ", p.a, " due to orientation difference ", diff);
                    continue;
                }

                float d = pointLineDistance(p, line);
                if (d < lineTol) {
                    float proj = project(p, line);
                    inliers.emplace_back(proj, &p);
                    //LOG_EXTENDED_DEBUG("Ransac: Point (", p.x, ", ", p.y, ") with angle ", p.a, " is an inlier (distance ", d, ", orientation diff ", diff, ")");
                }else{
                    //LOG_EXTENDED_DEBUG("Ransac: Ignoring point (", p.x, ", ", p.y, ") with angle ", p.a, " due to distance ", d);
                }
            }
        }
    }

    return false;

}