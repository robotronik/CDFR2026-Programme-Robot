#include "navigation/astar.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include "defs/constante.h"
#include "utils/logger.hpp"

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define MAX_OPEN_SIZE (HEIGHT * WIDTH)
#define FREE_SPACE 0

unsigned char costmap[HEIGHT][WIDTH];

typedef struct {
    int x, y;
    int g;  // Coût depuis le départ
    int h;  // Heuristique jusqu’à la cible
    int f;  // f = g + h
    int parent_x, parent_y;
    bool visited;
} Node;

Node nodes[HEIGHT][WIDTH];

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void print_costmap_around_point(int x, int y) {
    int half_size = 5; // Rayon du carré (10x10)
    for (int i = x - half_size; i <= x + half_size; i++) {
        for (int j = y - half_size; j <= y + half_size; j++) {
            if (i < 0 || i >= HEIGHT || j < 0 || j >= WIDTH) {
                printf("  ? "); // Hors limites
            } else if (costmap[i][j] == OBSTACLE_COST) {
                printf("  # "); // Obstacle
            } else if (costmap[i][j] == 0) {
                printf("  . "); // Espace libre
            } else {
                printf("%3d ", costmap[i][j]); // Coût
            }
        }
        printf("\n");
    }
    printf("\n");
}

// Récupère le chemin dans points[], retourne la longueur
int reconstruct_path_points(int start_x, int start_y, int goal_x, int goal_y, nav_pos_t *points, int max_points) {
    int length = 0;
    int x = goal_x;
    int y = goal_y;

    while (!(x == start_x && y == start_y)) {
        if (x < 0 || x >= HEIGHT || y < 0 || y >= WIDTH) {
            LOG_ERROR("Position (", x, ", ", y, ") hors limites dans reconstruct_path_points");
            break;  // évite le segfault
        }
        if (length >= max_points) break;
        points[length++] = (nav_pos_t){x, y};
        int px = nodes[x][y].parent_x;
        int py = nodes[x][y].parent_y;
        x = px;
        y = py;
    }
    if (length < max_points) {
        points[length++] = (nav_pos_t){start_x, start_y};
    }

    // Inverse le tableau (car actuellement du but vers départ)
    for (int i = 0; i < length/2; i++) {
        nav_pos_t tmp = points[i];
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

    nav_pos_t open[MAX_OPEN_SIZE];
    int open_size = 0;
    open[open_size++] = (nav_pos_t){start_x, start_y};

    while (open_size > 0) {
        // Trouver le noeud avec le plus petit f dans open
        int best_idx = 0;
        for (int i = 1; i < open_size; i++) {
            nav_pos_t p = open[i];
            if (nodes[p.x][p.y].f < nodes[open[best_idx].x][open[best_idx].y].f)
                best_idx = i;
        }

        nav_pos_t current = open[best_idx];

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
                    open[open_size++] = (nav_pos_t){nx, ny};
            }
        }
    }

    LOG_WARNING("Aucun chemin trouvé.");
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

int line_max_cost(nav_pos_t p1, nav_pos_t p2) {
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

int smooth_path(nav_pos_t *in_path, int in_length, nav_pos_t *out_path, int max_points) {
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

// Fonction de lissage par moyenne glissante
int smooth_path2(nav_pos_t *in_path, int in_length, position_t *out_path, int max_points, int window_size) {
    if (!in_path || !out_path || in_length <= 0 || max_points <= 0 || window_size <= 0 || window_size > in_length) {
        for (int i = 0; i < in_length && i < max_points; ++i) {
            out_path[i].x = in_path[i].x;
            out_path[i].y = in_path[i].y;
        }
        return in_length; // Copie directe du chemin
        return -1; // Paramètres invalides
    }

    int half_window = window_size / 2;
    int out_index = 0;

    for (int i = 0; i < in_length && out_index < max_points; ++i) {
        int x_sum = 0, y_sum = 0, cost_sum = 0;
        int count = 0;

        // Calcul de la moyenne dans la fenêtre centrée sur i
        for (int j = i - half_window; j <= i + half_window; ++j) {
            if (j >= 0 && j < in_length) {
                x_sum += in_path[j].x;
                y_sum += in_path[j].y;
                cost_sum += in_path[j].cost;
                ++count;
            }
        }

        // Moyenne entière arrondie
        out_path[out_index].x = 1.0*x_sum / count;
        out_path[out_index].y = 1.0*y_sum / count;
        

        ++out_index;
    }

    return out_index; // Nombre de points générés
}


void print_costmap_with_path(nav_pos_t *path, int path_len) {
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