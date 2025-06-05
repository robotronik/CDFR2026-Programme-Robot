#include "navigation/nav.h"
unsigned char costmap[HEIGHT][WIDTH];
Node nodes[HEIGHT][WIDTH];

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

// Récupère le chemin dans points[], retourne la longueur
int reconstruct_path_points(int start_x, int start_y, int goal_x, int goal_y, position_int *points, int max_points) {
    int length = 0;
    int x = goal_x;
    int y = goal_y;

    while (!(x == start_x && y == start_y)) {
        if (x < 0 || x >= HEIGHT || y < 0 || y >= WIDTH) {
            printf("Erreur: position (%d,%d) hors limites dans reconstruct_path_points\n", x, y);
            break;  // évite le segfault
        }
        if (length >= max_points) break;
        points[length++] = (position_int){x, y};
        int px = nodes[x][y].parent_x;
        int py = nodes[x][y].parent_y;
        x = px;
        y = py;
    }
    if (length < max_points) {
        points[length++] = (position_int){start_x, start_y};
    }

    // Inverse le tableau (car actuellement du but vers départ)
    for (int i = 0; i < length/2; i++) {
        position_int tmp = points[i];
        points[i] = points[length-1-i];
        points[length-1-i] = tmp;
    }
    return length;
}

void a_star(int start_x, int start_y, int goal_x, int goal_y) {
    for (int x = 0; x < HEIGHT; x++)
        for (int y = 0; y < WIDTH; y++) {
            nodes[x][y].x = x;
            nodes[x][y].y = y;
            nodes[x][y].g = INT_MAX;
            nodes[x][y].f = INT_MAX;
            nodes[x][y].visited = false;
        }

    nodes[start_x][start_y].g = 0;
    nodes[start_x][start_y].h = heuristic(start_x, start_y, goal_x, goal_y);
    nodes[start_x][start_y].f = nodes[start_x][start_y].h;

    position_int open[MAX_OPEN_SIZE];
    int open_size = 0;
    open[open_size++] = (position_int){start_x, start_y};

    while (open_size > 0) {
        // Trouver le noeud avec le plus petit f dans open
        int best_idx = 0;
        for (int i = 1; i < open_size; i++) {
            position_int p = open[i];
            if (nodes[p.x][p.y].f < nodes[open[best_idx].x][open[best_idx].y].f)
                best_idx = i;
        }

        position_int current = open[best_idx];

        // Retirer current de open
        open[best_idx] = open[--open_size];

        int cx = current.x, cy = current.y;
        if (cx == goal_x && cy == goal_y) {
            return;
        }

        nodes[cx][cy].visited = true;

        // Voisinage en 4 directions (NSEW)
        int directions[4][2] = {{1,0}, {-1,0}, {0,1}, {0,-1}};

        for (int d = 0; d < 4; d++) {
            int nx = cx + directions[d][0];
            int ny = cy + directions[d][1];

            if (nx < 0 || ny < 0 || nx >= HEIGHT || ny >= WIDTH)
                continue;

            if (costmap[nx][ny] >= OBSTACLE_COST)
                continue;  // Obstacle impassable

            if (nodes[nx][ny].visited)
                continue;

            int tentative_g = nodes[cx][cy].g + costmap[nx][ny] + 1;

            if (tentative_g < nodes[nx][ny].g) {
                nodes[nx][ny].g = tentative_g;
                nodes[nx][ny].h = heuristic(nx, ny, goal_x, goal_y);
                nodes[nx][ny].f = nodes[nx][ny].g + nodes[nx][ny].h;
                nodes[nx][ny].parent_x = cx;
                nodes[nx][ny].parent_y = cy;

                // Ajouter à open si pas déjà dedans
                bool in_open = false;
                for (int i = 0; i < open_size; i++)
                    if (open[i].x == nx && open[i].y == ny)
                        in_open = true;
                if (!in_open && open_size < MAX_OPEN_SIZE)
                    open[open_size++] = (position_int){nx, ny};
            }
        }
    }

    printf("Aucun chemin trouvé.\n");
}

void initialize_costmap() {
    int border = ROBOT_WIDTH / 2/RESOLUTION;

    for (int x = 0; x < HEIGHT; x++) {
        for (int y = 0; y < WIDTH; y++) {
            // Vérifie si la cellule est proche d’un bord
            if (x < border || x >= HEIGHT - border || y < border || y >= WIDTH - border)
                costmap[x][y] = OBSTACLE_COST;  // Bord = obstacle
            else
                costmap[x][y] = FREE_SPACE;
        }
    }
}


// Place un obstacle rectangulaire centré en (cx, cy), avec largeur et hauteur en mm, obstacle_radius_mm est la marge que tu donnes à la zone (pour un mur, 10cm devrait suffire)
void place_obstacle_rect_with_inflation(int cx, int cy, int height_mm, int width_mm, int obstacle_radius_mm) {
    cx = convert_x_to_index(cx);
    cy = convert_y_to_index(cy);
    int half_w_cells = height_mm / (2 * RESOLUTION);
    int half_h_cells = width_mm / (2 * RESOLUTION);

    int inflation_radius = (ROBOT_WIDTH / 2 + obstacle_radius_mm) / RESOLUTION;

    for (int x = cx - half_w_cells; x <= cx + half_w_cells; x++) {
        for (int y = cy - half_h_cells; y <= cy + half_h_cells; y++) {
            if (x < 0 || x >= HEIGHT || y < 0 || y >= WIDTH) continue;

            costmap[x][y] = OBSTACLE_COST;

            // Appliquer l'inflation autour
            for (int dx = -inflation_radius; dx <= inflation_radius; dx++) {
                for (int dy = -inflation_radius; dy <= inflation_radius; dy++) {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx < 0 || nx >= HEIGHT || ny < 0 || ny >= WIDTH) continue;

                    int distance = abs(dx) + abs(dy);
                    if (distance == 0 || distance > inflation_radius) continue;

                    int cost = OBSTACLE_COST - (distance * (OBSTACLE_COST / inflation_radius));
                    if (cost > costmap[nx][ny]) {
                        costmap[nx][ny] = cost;
                    }
                }
            }
        }
    }
}

void print_costmap() {
    for (int x = 0; x < HEIGHT; x++) {
        for (int y = 0; y < WIDTH; y++) {
            if (costmap[x][y] == 254) {
                printf("  X ");  // Afficher un 'X' pour le chemin
            } else if (costmap[x][y] == OBSTACLE_COST) {
                printf("  # ");  // Afficher '#' pour les obstacles
            } else if (costmap[x][y] == 0) {
                printf("  . ");  // Afficher '.' pour l'espace libre
            } else {
                printf("%3d ", costmap[x][y]);  // Sinon afficher le coût
            }
        }
        printf("\n");
    }
    printf("\n");
}

int line_max_cost(position_int p1, position_int p2) {
    int x0 = p1.x, y0 = p1.y;
    int x1 = p2.x, y1 = p2.y;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int max_cost = 0;

    while (true) {
        if (x0 < 0 || x0 >= HEIGHT || y0 < 0 || y0 >= WIDTH)
            return INT_MAX;

        int cost = costmap[x0][y0];
        if (cost > max_cost)
            max_cost = cost;

        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }

    return max_cost;
}

int smooth_path(position_int *in_path, int in_length, position_int *out_path, int max_points) {
    int out_len = 0;
    int i = 0;

    while (i < in_length) {
        out_path[out_len++] = in_path[i];
        if (out_len >= max_points) break;

        int best_j = i + 1;

        // Essayer d'aller aussi loin que possible sans dépasser un coût > 1
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

void print_costmap_with_path(position_int *path, int path_len) {
    for (int x = 0; x < HEIGHT; x++) {
        for (int y = 0; y < WIDTH; y++) {
            bool is_path = false;
            for (int i = 0; i < path_len; i++) {
                if (path[i].x == x && path[i].y == y) {
                    is_path = true;
                    break;
                }
            }
            if (is_path) {
                printf("  X ");  // Afficher 'X' pour les points du chemin
            } else if (costmap[x][y] == OBSTACLE_COST) {
                printf("  # ");  // Afficher '#' pour les obstacles
            }
            else if (costmap[x][y] == 0) {
                printf("  . ");  // Afficher '.' pour rien
            } else {
                printf("%3d ", costmap[x][y]);  // Afficher le coût sinon
            }
        }
        printf("\n");
    }
    printf("\n");
}

int convert_x_to_index(int x) {
    int offset = 1000; // Décalage pour convertir [-1000:1000] en [0:2000]
    int index = (x + offset) / RESOLUTION;
    if (index < 0 || index > WIDTH) {
        printf("Erreur: index hors limites index = (%d), x = (%d)\n", index,x);
        return -1; // Retourne une valeur invalide si hors limites
    }
    return index;
}
int convert_y_to_index(int y) {
    int offset = 1500; // Décalage pour convertir [-1000:1000] en [0:2000]
    int index = (y + offset) / RESOLUTION;
    if (index < 0 || index > WIDTH) {
        printf("Erreur: index hors limites index = (%d), y = (%d)\n", index, y);
        return -1; // Retourne une valeur invalide si hors limites
    }
    return index;
}

void convert_path_to_coordinates(position_int *path, int path_len) {
    for (int i = 0; i < path_len; i++) {
        path[i].x = path[i].x * RESOLUTION - 1000; // Conversion en coordonnées x
        path[i].y = path[i].y * RESOLUTION - 1500; // Conversion en coordonnées y
    }
}