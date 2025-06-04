#include "navigation/nav.h"
unsigned char costmap[HEIGHT][WIDTH];
Node nodes[HEIGHT][WIDTH];
int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// Récupère le chemin dans points[], retourne la longueur
int reconstruct_path_points(int start_x, int start_y, int goal_x, int goal_y, Point *points, int max_points) {
    int length = 0;
    int x = goal_x;
    int y = goal_y;

    while (!(x == start_x && y == start_y)) {
        if (length >= max_points) break;
        points[length++] = (Point){x, y};
        int px = nodes[y][x].parent_x;
        int py = nodes[y][x].parent_y;
        x = px;
        y = py;
    }
    if (length < max_points) {
        points[length++] = (Point){start_x, start_y};
    }

    // Inverse le tableau (car actuellement du but vers départ)
    for (int i = 0; i < length/2; i++) {
        Point tmp = points[i];
        points[i] = points[length-1-i];
        points[length-1-i] = tmp;
    }
    return length;
}


void a_star(int start_x, int start_y, int goal_x, int goal_y) {
    for (int y = 0; y < HEIGHT; y++)
        for (int x = 0; x < WIDTH; x++) {
            nodes[y][x].x = x;
            nodes[y][x].y = y;
            nodes[y][x].g = INT_MAX;
            nodes[y][x].f = INT_MAX;
            nodes[y][x].visited = false;
        }

    nodes[start_y][start_x].g = 0;
    nodes[start_y][start_x].h = heuristic(start_x, start_y, goal_x, goal_y);
    nodes[start_y][start_x].f = nodes[start_y][start_x].h;

    Point open[MAX_OPEN_SIZE];
    int open_size = 0;
    open[open_size++] = (Point){start_x, start_y};

    while (open_size > 0) {
        // Trouver le noeud avec le plus petit f dans open
        int best_idx = 0;
        for (int i = 1; i < open_size; i++) {
            Point p = open[i];
            if (nodes[p.y][p.x].f < nodes[open[best_idx].y][open[best_idx].x].f)
                best_idx = i;
        }

        Point current = open[best_idx];

        // Retirer current de open
        open[best_idx] = open[--open_size];

        int cx = current.x, cy = current.y;
        if (cx == goal_x && cy == goal_y) {
            return;
        }

        nodes[cy][cx].visited = true;

        // Voisinage en 4 directions (NSEW)
        int directions[4][2] = {{1,0}, {-1,0}, {0,1}, {0,-1}};

        for (int d = 0; d < 4; d++) {
            int nx = cx + directions[d][0];
            int ny = cy + directions[d][1];

            if (nx < 0 || ny < 0 || nx >= WIDTH || ny >= HEIGHT)
                continue;

            if (costmap[ny][nx] >= OBSTACLE_COST)
                continue;  // Obstacle impassable

            if (nodes[ny][nx].visited)
                continue;

            int tentative_g = nodes[cy][cx].g + costmap[ny][nx] + 1;

            if (tentative_g < nodes[ny][nx].g) {
                nodes[ny][nx].g = tentative_g;
                nodes[ny][nx].h = heuristic(nx, ny, goal_x, goal_y);
                nodes[ny][nx].f = nodes[ny][nx].g + nodes[ny][nx].h;
                nodes[ny][nx].parent_x = cx;
                nodes[ny][nx].parent_y = cy;

                // Ajouter à open si pas déjà dedans
                bool in_open = false;
                for (int i = 0; i < open_size; i++)
                    if (open[i].x == nx && open[i].y == ny)
                        in_open = true;
                if (!in_open && open_size < MAX_OPEN_SIZE)
                    open[open_size++] = (Point){nx, ny};
            }
        }
    }

    printf("Aucun chemin trouvé.\n");
}




void initialize_costmap() {
    for (int y = 0; y < HEIGHT; y++)
        for (int x = 0; x < WIDTH; x++)
            costmap[y][x] = FREE_SPACE;
}

// Place un obstacle rectangulaire centré en (cx, cy), avec largeur et hauteur en mm, obstacle_radius_mm est la marge que tu donnes à la zone (pour un mur, 10cm devrait suffire)
void place_obstacle_rect_with_inflation(int cx_mm, int cy_mm, int width_mm, int height_mm, int obstacle_radius_mm) {
    int origin_x = WIDTH / 2;
    int origin_y = HEIGHT / 2;

    // Convertir (cx_mm, cy_mm) en indices de la grille
    int grid_cx = origin_x + cx_mm / RESOLUTION;
    int grid_cy = origin_y - cy_mm / RESOLUTION;  // Y inversé si haut = -y

    int half_w_cells = width_mm / (2 * RESOLUTION);
    int half_h_cells = height_mm / (2 * RESOLUTION);

    int inflation_radius = (ROBOT_WIDTH / 2 + obstacle_radius_mm) / RESOLUTION;

    for (int y = grid_cy - half_h_cells; y <= grid_cy + half_h_cells; y++) {
        for (int x = grid_cx - half_w_cells; x <= grid_cx + half_w_cells; x++) {
            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) continue;

            costmap[y][x] = OBSTACLE_COST;

            // Appliquer l'inflation autour
            for (int dy = -inflation_radius; dy <= inflation_radius; dy++) {
                for (int dx = -inflation_radius; dx <= inflation_radius; dx++) {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx < 0 || nx >= WIDTH || ny < 0 || ny >= HEIGHT) continue;

                    int distance = abs(dx) + abs(dy);
                    if (distance == 0 || distance > inflation_radius) continue;

                    int cost = OBSTACLE_COST - (distance * (OBSTACLE_COST / inflation_radius));
                    if (cost > costmap[ny][nx]) {
                        costmap[ny][nx] = cost;
                    }
                }
            }
        }
    }
}

void print_costmap() {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            if (costmap[y][x] == 254) {
                printf("  X ");  // Afficher un '*' pour le chemin
            } else if (costmap[y][x] == OBSTACLE_COST) {
                printf("  # ");  // Afficher '#' pour les obstacles (optionnel)
            } else if (costmap[y][x] == 0) {
                printf("  . ");  // Afficher '.' 
            } else {
                printf("%3d ", costmap[y][x]);  // Sinon afficher le coût
            }
        }
        printf("\n");
    }
    printf("\n");
}

int line_max_cost(Point p1, Point p2) {
    int x0 = p1.x, y0 = p1.y;
    int x1 = p2.x, y1 = p2.y;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int max_cost = 0;

    while (true) {
        if (x0 < 0 || x0 >= WIDTH || y0 < 0 || y0 >= HEIGHT)
            return INT_MAX;

        int cost = costmap[y0][x0];
        if (cost > max_cost)
            max_cost = cost;

        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }

    return max_cost;
}

int smooth_path(Point *in_path, int in_length, Point *out_path, int max_points) {
    int out_len = 0;
    int i = 0;

    while (i < in_length) {
        out_path[out_len++] = in_path[i];
        if (out_len >= max_points) break;

        int best_j = i + 1;

        // Essayer d'aller aussi loin que possible sans dépasser un coût > 10
        for (int j = in_length - 1; j > i; j--) {
            if (line_max_cost(in_path[i], in_path[j]) <= 1) {
                best_j = j;
                break;
            }
        }

        i = best_j;
    }

    return out_len;
}

void print_costmap_with_path(Point *path, int path_len) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            bool is_path = false;
            for (int i = 0; i < path_len; i++) {
                if (path[i].x == x && path[i].y == y) {
                    is_path = true;
                    break;
                }
            }
            if (is_path) {
                printf("  X ");  // Afficher 'X' pour les points du chemin
            } else if (costmap[y][x] == OBSTACLE_COST) {
                printf("  # ");  // Afficher '#' pour les obstacles
            }
            else if (costmap[y][x] == 0) {
                printf("  . ");  // Afficher '.' pour rien
            } else {
                printf("%3d ", costmap[y][x]);  // Afficher le coût sinon
            }
        }
        printf("\n");
    }
    printf("\n");
}