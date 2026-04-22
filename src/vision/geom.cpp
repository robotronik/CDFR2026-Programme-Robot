#include "vision/geom.hpp"

// normalisation angle [-180, 180]
float angleDiff(float a, float b) {
    float d = fmodf(a - b + 180.0f, 360.0f) - 180.0f;
    return std::abs(d);
}

// distance point-droite 2D
float pointLineDistance(const block_t& p, const Line& l) {
    float vx = p.x - l.x;
    float vy = p.y - l.y;

    float cross = std::abs(vx * l.dy - vy * l.dx);
    return cross; // car dir normalisée
}

// projection scalaire
float project(const block_t& p, const Line& l) {
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

float cross2d(float x1, float y1, float x2, float y2) {
    return x1 * y2 - y1 * x2;
}

bool checkSegment(float px, float py, float tx, float ty) {
    float len = std::sqrt(tx*tx + ty*ty);
    if (len < 1e-6f) return false;
    
    float ux = tx / len;
    float uy = ty / len;
    
    // Projection scalaire du point sur le segment
    float t = px * ux + py * uy;
    
    // On vérifie si on est devant le robot et avant la cible (marge incluse)
    if (t > -ROBOT_RADIUS && t < len - TARGET_MARGIN) {
        float proj_x = t * ux;
        float proj_y = t * uy;
        float d = std::sqrt((px - proj_x)*(px - proj_x) + (py - proj_y)*(py - proj_y));
        return d < CORRIDOR_THRESHOLD;
    }
    return false;
};

namespace {
    // Calcule les 4 coins d'un rectangle orienté (OBB)
    void getOBBCorners(float cx, float cy, float w, float h, float angle_deg, float corners[4][2]) {
        float a_rad = angle_deg * M_PI / 180.0f;
        float cos_a = std::cos(a_rad);
        float sin_a = std::sin(a_rad);
        float hw = w / 2.0f;
        float hh = h / 2.0f;
        
        // Coordonnées locales des 4 coins
        float dx[4] = {-hw, hw, hw, -hw};
        float dy[4] = {-hh, -hh, hh, hh};
        
        for (int i = 0; i < 4; i++) {
            corners[i][0] = cx + dx[i] * cos_a - dy[i] * sin_a;
            corners[i][1] = cy + dx[i] * sin_a + dy[i] * cos_a;
        }
    }

    // Théorème des Axes Séparateurs (SAT) pour détecter une collision entre 2 rectangles
    bool checkOBBCollision(const float c1[4][2], const float c2[4][2]) {
        // Les 4 axes potentiels de séparation (2 pour chaque rectangle)
        float axes[4][2] = {
            {c1[1][0] - c1[0][0], c1[1][1] - c1[0][1]},
            {c1[2][0] - c1[1][0], c1[2][1] - c1[1][1]},
            {c2[1][0] - c2[0][0], c2[1][1] - c2[0][1]},
            {c2[2][0] - c2[1][0], c2[2][1] - c2[1][1]}
        };
        
        for (int i = 0; i < 4; i++) {
            float ax = axes[i][0];
            float ay = axes[i][1];
            float len = std::sqrt(ax*ax + ay*ay);
            if (len < 1e-6f) continue;
            ax /= len; ay /= len; // Normalisation de l'axe
            
            // Projection du rectangle 1 sur l'axe
            float min1 = 1e9f, max1 = -1e9f;
            for (int j = 0; j < 4; j++) {
                float proj = c1[j][0] * ax + c1[j][1] * ay;
                min1 = std::min(min1, proj);
                max1 = std::max(max1, proj);
            }
            
            // Projection du rectangle 2 sur l'axe
            float min2 = 1e9f, max2 = -1e9f;
            for (int j = 0; j < 4; j++) {
                float proj = c2[j][0] * ax + c2[j][1] * ay;
                min2 = std::min(min2, proj);
                max2 = std::max(max2, proj);
            }
            
            // S'il y a un espace entre les projections, les rectangles ne se touchent pas
            if (max1 < min2 || max2 < min1) {
                return false;
            }
        }
        return true; // Aucun axe séparateur trouvé = collision
    }
}

bool blockInFrontInterface(const std::vector<std::tuple<float, const block_t*, bool>>& choosen, const std::vector<block_t>& points) {
    std::vector<const block_t*> block_temps;

    block_temps.reserve(choosen.size()); 
    
    for (const auto& c : choosen) {
        block_temps.push_back(std::get<1>(c));
    }
    return blockInFront(block_temps, points);
}

bool blockInFrontInterface(const std::vector<std::pair<float, const block_t*>>& choosen, const std::vector<block_t>& points) {
    std::vector<const block_t*> block_temps;

    block_temps.reserve(choosen.size()); 
    
    for (const auto& c : choosen) {
        block_temps.push_back(c.second);
    }
    return blockInFront(block_temps, points);
}

bool blockInFront(const std::vector<const block_t*>& choosen, const std::vector<block_t>& points) {
    if (choosen.empty()) return false;
    if (choosen.size() == points.size()) return false;

    // Les deux blocs extrêmes qui définissent notre zone
    const block_t* blockA = choosen.front();
    const block_t* blockB = choosen.back();

    float ax = static_cast<float>(blockA->x);
    float ay = static_cast<float>(blockA->y);
    float bx = static_cast<float>(blockB->x);
    float by = static_cast<float>(blockB->y);

    // Vecteur du segment AB
    float abx = bx - ax;
    float aby = by - ay;
    float len_ab = std::sqrt(abx*abx + aby*aby);

    bool is_single_line = (len_ab < 1e-6f); // Vrai si A et B sont quasiment au même endroit (le même block)

    // Calcul de la normale à AB pointant vers l'origine (le robot)
    float nx = 0, ny = 0;
    if (!is_single_line) {
        nx = -aby;
        ny = abx;
        // On s'assure que la normale pointe vers l'origine (0,0)
        // Produit scalaire entre la normale et le vecteur A->Origine (-ax, -ay)
        if ((nx * (-ax) + ny * (-ay)) < 0) {
            nx = -nx;
            ny = -ny;
        }
        float len_n = std::sqrt(nx*nx + ny*ny);
        nx /= len_n;
        ny /= len_n;
    }

    for (const block_t& p : points) { 
        // 1. Vérifier si 'p' est une de nos cibles
        bool is_chosen = false;
        for (const block_t* c : choosen) {
            if (&p == c) {
                is_chosen = true;
                break;
            }
        }
        if (is_chosen) continue; 

        float px = static_cast<float>(p.x);
        float py = static_cast<float>(p.y);

        // Si A et B sont confondus (ex: 1 seul bloc choisi), on fait juste un test de segment classique
        if (is_single_line) {
            if (checkSegment(px, py, ax, ay)) {
                return true;
            }
            continue;
        }

        // --- Vérification géométrique complexe (Triangle OAB et bordures) ---

        // A. Vérifier si le point est dans le "cône" infini formé par l'origine, A et B
        float cab = cross2d(ax, ay, bx, by);
        float cross_AP = cross2d(ax, ay, px, py);
        float cross_BP = cross2d(bx, by, px, py);

        bool is_inside_cone = false;
        if (cab >= 0) {
            is_inside_cone = (cross_AP >= 0 && cross_BP <= 0);
        } else {
            is_inside_cone = (cross_BP >= 0 && cross_AP <= 0);
        }

        // B. Si le point est dans le cône, est-il bien "devant" la ligne AB ?
        if (is_inside_cone) {
            // Distance du point P à la droite AB (orientée vers l'origine)
            float dist_to_AB = (px - ax) * nx + (py - ay) * ny;
            
            // Si la distance est supérieure à la marge, le point est sur la route du robot
            if (dist_to_AB > TARGET_MARGIN) {
                return true; 
            }
        }

        // C. Si le point n'est pas dans le triangle strict, il pourrait quand même "déborder" 
        // sur les trajectoires latérales du robot à cause de son diamètre (CORRIDOR_THRESHOLD)
        if (checkSegment(px, py, ax, ay) || checkSegment(px, py, bx, by)) {
            return true;
        }
    }

    return false;
}

block_t interfacePlacePoussoir(const std::vector<std::pair<float, const block_t*>>& choosen, const std::vector<block_t>& points){
    std::vector<const block_t*> block_temps;

    block_temps.reserve(choosen.size()); 
    
    for (const auto& c : choosen) {
        block_temps.push_back(c.second);
    }
    return placePoussoir(block_temps, points);
}

block_t placePoussoir(const std::vector<const block_t*>& choosen, const std::vector<block_t>& points) {
    block_t best_pusher = block_t{0,0,0, false};
    if (choosen.empty()) return best_pusher;

    // Le bloc le plus à "droite" (fin de la sélection)
    const block_t* target = choosen.back();
    
    // Détermination de l'axe de la droite
    float ux, uy;
    if (choosen.size() >= 2) {
        ux = target->x - choosen.front()->x;
        uy = target->y - choosen.front()->y;
    } else {
        // Fallback s'il n'y a qu'un seul bloc (on se fie à son angle)
        float a_rad = target->a * M_PI / 180.0f;
        ux = std::cos(a_rad);
        uy = std::sin(a_rad);
    }

    float len = std::sqrt(ux*ux + uy*uy);
    if (len < 1e-6f) {
        float a_rad = target->a * M_PI / 180.0f;
        ux = std::cos(a_rad);
        uy = std::sin(a_rad);
    } else {
        ux /= len; 
        uy /= len;
    }
    
    // Vecteur normal (pour s'écarter de la droite si besoin)
    float vx = -uy; 
    float vy = ux;

    // Angle de la ligne
    float line_angle = std::atan2(uy, ux) * 180.0f / M_PI;

    float min_cost = 1e12f;

    // ---------------------------------------------------------
    // EXPLORATION DE L'ESPACE DES SOLUTIONS
    // ---------------------------------------------------------
    // d : distance le long de la ligne [1cm à 8cm]
    // h : décalage orthogonal par rapport à la ligne [-15cm à 15cm]
    // da : angle du poussoir par rapport à la ligne (centré sur 90°)
    
    for (float h = -15.0f; h <= 15.0f; h += 0.5f) {
        for (float da = 45.0f; da <= 135.0f; da += 5.0f) { // de 45° à 135° (idéalement 90)
            for (float d = 1.0f; d <= 8.0f; d += 0.5f) {
                
                // Position candidate du centre du poussoir
                float px = target->x + d * ux + h * vx;
                float py = target->y + d * uy + h * vy;
                float pa = line_angle + da;

                // 1. Génération de l'encombrement du poussoir (X=2, Y=15)
                float pusher_corners[4][2];
                getOBBCorners(px, py, 2.0f, 15.0f, pa, pusher_corners);

                // 2. Vérification des collisions avec tous les blocs non sélectionnés
                bool collision = false;
                for (const auto& pt : points) {
                    bool is_chosen = false;
                    for (const auto& c : choosen) {
                        if (&pt == c) { is_chosen = true; break; }
                    }
                    if (is_chosen) continue; // On ignore les blocs ciblés

                    // Encombrement d'un bloc classique (X=5, Y=15)
                    float block_corners[4][2];
                    getOBBCorners(pt.x, pt.y, 5.0f, 15.0f, pt.a, block_corners);

                    if (checkOBBCollision(pusher_corners, block_corners)) {
                        collision = true;
                        break;
                    }
                }

                // 3. Évaluation du candidat s'il n'y a pas de collision
                if (!collision) {
                    // Calcul du coût. Ordre d'importance :
                    // - Être le plus proche de la droite possible (fort poids sur h)
                    // - Être le plus perpendiculaire possible (poids sur da)
                    // - Rapprocher ou ajuster la distance d si nécessaire (faible poids)
                    float cost = std::abs(h) * 1000.0f + std::abs(da - 90.0f) * 10.0f + std::abs(d - 4.5f);
                    
                    if (cost < min_cost) {
                        min_cost = cost;
                        best_pusher.x = px;
                        best_pusher.y = py;
                        
                        // Normalisation de l'angle [-180, 180]
                        float norm_a = std::fmod(pa, 360.0f);
                        if (norm_a > 180.0f) norm_a -= 360.0f;
                        if (norm_a < -180.0f) norm_a += 360.0f;
                        
                        best_pusher.a = norm_a;
                        best_pusher.color = target->color;
                    }
                }
            }
        }
    }

    return best_pusher;
}