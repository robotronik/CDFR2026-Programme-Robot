#include "vision/geom.hpp"

// Normalisation de l'angle pour des objets symétriques (modulo 180°)
// Renvoie l'écart absolu le plus court, toujours compris entre [0, 90]
float angleDiff(float a, float b) {
    // Calcul de la différence brute modulo 180
    float d = std::fmod(a - b, 180.0f);
    
    // std::fmod peut renvoyer une valeur négative en C++,
    // on la ramène donc systématiquement dans l'intervalle [0, 180)
    if (d < 0.0f) {
        d += 180.0f;
    }
    
    // Comme le bloc est symétrique, l'écart maximal est de 90°
    // (ex: un écart de 170° équivaut à un écart réel de 10°)
    if (d > 90.0f) {
        d = 180.0f - d;
    }
    
    return d;
}

bool acceptableAngle(const block_t* b1, const block_t* b2, float angleTol){
    float dx = b2->x - b1->x;
    float dy = b2->y - b1->y;

    if (std::abs(dx) < 1e-6f && std::abs(dy) < 1e-6f) {
        return false; 
    }

    // Calcul de l'angle de la ligne (en degrés)
    float lineAngle = std::atan2(dy, dx) * 180.0f / M_PI;
    
    // Vérification de l'angle de b1 par rapport à la ligne
    return angleDiff(b1->a, lineAngle) < angleTol;
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
    void getOBBCorners(float cx, float cy, float w, float h, float angle_deg, block_t corners[4]) {
        float a_rad = angle_deg * M_PI / 180.0f;
        float cos_a = std::cos(a_rad);
        float sin_a = std::sin(a_rad);
        float hw = w / 2.0f;
        float hh = h / 2.0f;
        
        // Coordonnées locales des 4 coins
        float dx[4] = {-hw, hw, hw, -hw};
        float dy[4] = {-hh, -hh, hh, hh};
        
        for (int i = 0; i < 4; i++) {
            corners[i].x = cx + dx[i] * cos_a - dy[i] * sin_a;
            corners[i].y = cy + dx[i] * sin_a + dy[i] * cos_a;
            corners[i].a = angle_deg;
            corners[i].color = false;
        }
    }

    // Théorème des Axes Séparateurs (SAT) pour détecter une collision entre 2 rectangles
    bool checkOBBCollision(const block_t c1[4], const block_t c2[4]) {
        // Les 4 axes potentiels de séparation (2 pour chaque rectangle)
        float axes[4][2] = {
            {c1[1].x - c1[0].x, c1[1].y - c1[0].y},
            {c1[2].x - c1[1].x, c1[2].y - c1[1].y},
            {c2[1].x - c2[0].x, c2[1].y - c2[0].y},
            {c2[2].x - c2[1].x, c2[2].y - c2[1].y}
        };
        
        for (int i = 0; i < 4; i++) {
            float ax = axes[i][0];
            float ay = axes[i][1];
            
            // Projection du rectangle 1 sur l'axe
            float min1 = 1e9f, max1 = -1e9f;
            for (int j = 0; j < 4; j++) {
                float proj = c1[j].x * ax + c1[j].y * ay;
                min1 = std::min(min1, proj);
                max1 = std::max(max1, proj);
            }
            
            // Projection du rectangle 2 sur l'axe
            float min2 = 1e9f, max2 = -1e9f;
            for (int j = 0; j < 4; j++) {
                float proj = c2[j].x * ax + c2[j].y * ay;
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

block_t transformRobotCoordToTableCoord(block_t startPos, block_t objectivePos, bool steal){
    //Traitement pour passer dans les coordonnées de la table
    // Décalage pour le centre du robot
    double rad_objective_a = objectivePos.a * M_PI / 180.0;
    //LOG_EXTENDED_DEBUG("Position avant correction du décalage : { x = ", tmp_x, ", y = ", tmp_y, ", a = ", tmp_a, " }");
    //LOG_EXTENDED_DEBUG("Décalage appliqué : { sin = ", OFFSET_STOCK * mult_param * sin(rad_tmp_a), ", cos = ", OFFSET_STOCK * mult_param * cos(rad_tmp_a), " }");
    const double off_s = 85; // Augmenter pour se rapprocher
    objectivePos.x += OFFSET_CAM_X - (OFFSET_STOCK - off_s) * cos(rad_objective_a) + (steal ? STEAL_OFFSET_X : 0);
    objectivePos.y += OFFSET_CAM_Y + OFFSET_CLAW_Y - (OFFSET_STOCK - off_s) * sin(rad_objective_a) + (steal ? STEAL_OFFSET_Y : 0);
    //LOG_EXTENDED_DEBUG("Position après correction du décalage : { x = ", objectivePos.x, ", y = ", objectivePos.y, ", a = ", objectivePos.a, " }");


    //projection dans le repère de la table:
    double a_rad = (startPos.a) * M_PI / 180.0;
    double cos_a = cos(a_rad);
    double sin_a = sin(a_rad);
    startPos.x += objectivePos.x * cos_a - objectivePos.y * sin_a;
    startPos.y += objectivePos.x * sin_a + objectivePos.y * cos_a;
    startPos.a += objectivePos.a;
    return startPos;
}

/*************GESTION DETECTION ROBOT DANS MUR********************/
bool pointDansMur(block_t p){
    return (p.x < MIN_X_TABLE || p.x > MAX_X_TABLE || p.y < MIN_Y_TABLE || p.y > MAX_Y_TABLE);
}
// rotation + translation
block_t transformPointFromLocalToWorld(block_t localPoint, block_t robotPos){
    block_t worldPoint;
    double angleRad = robotPos.a * M_PI / 180.0;

    worldPoint.x = robotPos.x + localPoint.x * cos(angleRad) - localPoint.y * sin(angleRad);
    worldPoint.y = robotPos.y + localPoint.x * sin(angleRad) + localPoint.y * cos(angleRad);
    return worldPoint;
}

void getRobotEdge(block_t *points, block_t robotPos){
    block_t local[5];

    local[0] = (block_t){-ROBOT_RADIUS, 0}; // arrière
    local[1] = (block_t){ROBOT_RADIUS / 2,  ROBOT_RADIUS * sqrt(3) / 2};  // pointe gauche
    local[2] = (block_t){ROBOT_RADIUS / 2, -ROBOT_RADIUS * sqrt(3) / 2};  // point droite
    local[3] = (block_t){ROBOT_RADIUS / 2 + CLAWS_LENGHT,  CLAWS_LENGHT/2};  // pointe claw gauche
    local[4] = (block_t){ROBOT_RADIUS / 2 + CLAWS_LENGHT, -CLAWS_LENGHT/2};  // pointe claw droite

    for (int i = 0; i < 5; i++)
        points[i] = transformPointFromLocalToWorld(local[i], robotPos);
}

bool isRobotInWall(block_t robotPos){
    block_t robot[5];
    getRobotEdge(robot,robotPos);
    for(int i = 0; i < 5; i++) if(pointDansMur(robot[i])) return true;
    return false;
}


/************GESTION BLOCK IN FRONT**************/
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

/*************GESTION PLACE POUSSOIR********************/
block_t placePoussoir(const std::vector<const block_t*>& choosen, const std::vector<block_t>& points, block_t robotPos) {
    // Par défaut on initialise le bloc. S'il n'y a pas la place, ses coordonnées vaudront 0 et la couleur sera différente
    block_t best_pusher;
    best_pusher.color = false;
    if (choosen.empty()) return best_pusher;

    // Le bloc le plus à "droite" (fin de la sélection)
    const block_t* target = choosen.back();
    
    // 1. Détermination de l'axe de la droite
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

    float line_angle = std::atan2(uy, ux) * 180.0f / M_PI;
    //LOG_DEBUG("Placing pusher for block at (", target->x, ", ", target->y, ") with angle ", target->a);
    //LOG_DEBUG("Approach line angle is ", line_angle);
    // On centre la droite sur le bloc cible (target) pour que la projection de référence soit à ~0
    Line solution_line = Line{static_cast<float>(target->x), static_cast<float>(target->y), ux, uy};

    // 2. Détermine le maximum des projections des coins du block le plus à droite
    block_t right_block_corners[4];
    getOBBCorners(target->x, target->y, 50.0f, 150.0f, target->a, right_block_corners);

    float max_proj = -1e12f; // Initialisé en négatif au cas où la projection tombe derrière
    for (const auto& corner : right_block_corners) {
        float proj = project(corner, solution_line);
        if (proj > max_proj) {
            max_proj = proj;
        }
    }
    /*
    LOG_DEBUG("Max projection of target block corners on approach line is ", max_proj);
    LOG_DEBUG("Target block corners projections: ", 
        project(right_block_corners[0], solution_line), ", ",
        project(right_block_corners[1], solution_line), ", ",
        project(right_block_corners[2], solution_line), ", ",
        project(right_block_corners[3], solution_line));
    */
    // 3. Détermination de l'écart minimum entre le block le plus à droite et les obstacles DEVANT
    float min_gap = 1e12f;

    for (const auto& pt : points) {
        bool is_chosen = false;
        for (const auto& c : choosen) {
            if (&pt == c) { is_chosen = true; break; }
        }
        if (is_chosen) continue; // On ignore les blocs ciblés

        block_t obs_corners[4];
        // Encombrement de l'obstacle
        getOBBCorners(pt.x, pt.y, 50.0f, 150.0f, pt.a, obs_corners);

        for (int i = 0; i < 4; i++) {
            // Si le coin de l'obstacle est trop loin latéralement de la ligne d'approche, il n'est pas gênant
            if (pointLineDistance(obs_corners[i], solution_line) > MAX_DISTANCE_FROM_BLOCK) continue;
            
            float proj = project(obs_corners[i], solution_line);
            
            // On s'intéresse uniquement aux coins d'obstacles qui sont STRICTEMENT DEVANT le bloc max
            if (proj > max_proj) {
                float gap = proj - max_proj;
                if (gap < min_gap) {
                    min_gap = gap;
                }
            }
        }
    }

    // 4. Premier placement en angle final
    // Si l'espace est d'au moins 50mm, ou qu'il n'y a aucun obstacle (min_gap resté à 1e12)
    float placement;
    if (min_gap > 50.0f) {
        //LOG_DEBUG("Sufficient gap of ", min_gap, "mm found in front of the target block. Placing pusher in front.");
        // Le centre du poussoir doit être à +50mm du coin le plus avancé du bloc
        placement = max_proj + 50.0f;
    } else {
        //LOG_DEBUG("Not enough gap (", min_gap, "mm) in front of the target block. Placing pusher on the side.");
        // Pas assez de place devant : on place le poussoir sur la ligne, 
        // à 5 cm du centre du bloc cible, en le gardant parallèle.
        placement = 50.0f;
    }

    // Reconversion de l'avancée scalaire vers les coordonnées (X,Y) sur la carte
    best_pusher.x = solution_line.x + placement * solution_line.dx;
    best_pusher.y = solution_line.y + placement * solution_line.dy;

    // Angle perpendiculaire (+90°)
    float pa = line_angle + 90.0f;
    best_pusher.a = pa;
    //LOG_DEBUG("Placing pusher at (", best_pusher.x, ", ", best_pusher.y, ") with angle ", best_pusher.a);

    // 5. Vérification que le placement n'est pas dans un mur (cas extrême) ou dans un bloc
    bool placement_valid = false;
    while (pointLineDistance(best_pusher, solution_line) < MAX_DISTANCE_FROM_BLOCK)
    {
        if(!isRobotInWall(transformRobotCoordToTableCoord(best_pusher, robotPos, true))){
            placement_valid = true; // Placement valide tant que le poussoir n'est pas dans un block
            block_t pusher_corners[4];
            getOBBCorners(best_pusher.x, best_pusher.y, 30.0f, 150.0f, best_pusher.a, pusher_corners);
            for(const auto& pt : points){
                block_t obs_corners[4];
                getOBBCorners(pt.x, pt.y, 50.0f, 150.0f, pt.a, obs_corners);
                if(checkOBBCollision(pusher_corners, obs_corners)){
                    LOG_DEBUG("Pusher placement at (", best_pusher.x, ", ", best_pusher.y, ") with angle ", best_pusher.a, " is in block.");
                    placement_valid = false;
                    break;
                }
            }
            if(placement_valid){
                best_pusher.color = true; // Placement valide
                return best_pusher; // Placement valide, on sort de la boucle
            }
        }else{
            LOG_DEBUG("Pusher placement at (", best_pusher.x, ", ", best_pusher.y, ") with angle ", best_pusher.a, " is in wall.");
        }
        // Conversion de l'angle du poussoir en radians
        float angle_rad = best_pusher.a * M_PI / 180.0f;

        // On recule de 5mm sur l'axe du poussoir (opposé à son orientation)
        best_pusher.x -= 5.0f * std::cos(angle_rad); 
        best_pusher.y -= 5.0f * std::sin(angle_rad);
        LOG_DEBUG("Line distance ", pointLineDistance(best_pusher, solution_line));
    }
    best_pusher.color = false; // Si on sort de la boucle, c'est que le placement est invalide
    return best_pusher;
}

block_t interfacePlacePoussoir(const std::vector<std::pair<float, const block_t*>>& choosen, const std::vector<block_t>& points, block_t robotPos){
    std::vector<const block_t*> block_temps;

    block_temps.reserve(choosen.size()); 
    
    for (const auto& c : choosen) {
        block_temps.push_back(c.second);
    }
    return placePoussoir(block_temps, points, robotPos);
}

block_t interfacePlacePoussoir(const std::vector<std::tuple<float, const block_t*, bool>>& choosen, const std::vector<block_t>& points, block_t robotPos){
    std::vector<const block_t*> block_temps;

    block_temps.reserve(choosen.size()); 
    
    for (const auto& c : choosen) {
        block_temps.push_back(std::get<1>(c));
    }
    return placePoussoir(block_temps, points, robotPos);
}
