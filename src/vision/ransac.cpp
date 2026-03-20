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

bool findGroupRANSAC2D(
    const std::vector<block_t>& points,
    std::vector<block_t>& bestGroup,
    size_t max_blocks,
    int maxIterations,
    float lineTol,
    float spacing,
    float spacingTol,
    float angleTol // tolérance orientation
) {
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, points.size() - 1);

    for (int iter = 0; iter < maxIterations; iter++) {

        // 1. tirer 2 points
        int i = dist(rng);
        int j = dist(rng);
        if (i == j) continue;

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
                      return a.second->y > b.second->y;
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
                return true; // early exit if solution found
            }
        }
    }

    return false;
}