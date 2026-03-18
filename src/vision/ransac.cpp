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
    if (points.size() < 4) return false;

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, points.size() - 1);

    for (int iter = 0; iter < maxIterations; ++iter) {

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
            if (angleDiff(p.a + 90, lineAngle) > angleTol) // à tester angle tag pas dans l'axe de l'objet
                continue;

            float d = pointLineDistance(p, line);
            if (d < lineTol) {
                float proj = project(p, line);
                inliers.emplace_back(proj, &p);
            }
        }

        if (inliers.size() < max_blocks) continue;

        std::sort(inliers.begin(), inliers.end(),
                  [](const auto& a, const auto& b) {
                      return a.first < b.first;
                  });

        // vérification de l'espacement
        for (size_t k = 0; k + max_blocks - 1 < inliers.size(); ++k) {
            bool status = true;

            for(int spaces = 1; spaces < max_blocks; spaces ++ ){
                float d1 = inliers[k + spaces + 1].first - inliers[k + spaces].first;
                if(std::abs(d1 - spacing) < spacingTol){
                    continue;
                }else{
                    status = false;
                    break;
                }
            }

            if (status) {
                for(size_t _ = 0 ; _ < max_blocks; _++){
                    bestGroup.push_back(*inliers[k + max_blocks].second);
                }
                return true; // early exit if solution found
            }
        }
    }

    return false;
}