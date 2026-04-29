#include "navigation/astar.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h> // memset
#include "utils/logger.hpp"

typedef struct {
    int g, f;
    position_int_t parent;
    bool visited;
} Node;

unsigned char costmap[AS_HEIGHT][AS_WIDTH];
Node nodes[AS_HEIGHT][AS_WIDTH];

inline int heuristic(position_int_t a, position_int_t b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

inline bool is_position_int_invalid(position_int_t p){
    return (p.x < 0 || p.x >= AS_HEIGHT || p.y < 0 || p.y >= AS_WIDTH);
}

inline bool is_position_int_equal(position_int_t a, position_int_t b){
    return (a.x == b.x && a.y == b.y);
}

void astar_initialize_costmap(){
    memset(costmap, FREE_SPACE, sizeof(costmap));
}

int astar_pathfind(position_int_t start, position_int_t goal, position_int_t path[]) {
    if (is_position_int_invalid(start)) return -1;
    if (is_position_int_invalid(goal)) return -1;

    // Initialize nodes
    for (int x = 0; x < AS_HEIGHT; x++) {
        for (int y = 0; y < AS_WIDTH; y++) {
            nodes[x][y].g = INT_MAX;
            nodes[x][y].f = INT_MAX;
            nodes[x][y].visited = false;
            nodes[x][y].parent = {-1, -1};
        }
    }

    // Start search from the goal
    nodes[goal.x][goal.y].g = 0;
    nodes[goal.x][goal.y].f = heuristic(goal, start);

    position_int_t open[AS_HEIGHT * AS_WIDTH];
    int open_size = 0;
    open[open_size++] = goal;
    int dirs[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    while (open_size > 0) {
        // Find the node with the lowest f-score
        int best = 0;
        for (int i = 1; i < open_size; i++) {
            if (nodes[open[i].x][open[i].y].f < nodes[open[best].x][open[best].y].f)
                best = i;
        }

        position_int_t curr = open[best];
        open[best] = open[--open_size];
        if (is_position_int_equal(curr, start)) break;

        nodes[curr.x][curr.y].visited = true;

        for (int d = 0; d < 4; d++) {
            position_int_t next = curr;
            next.x += dirs[d][0];
            next.y += dirs[d][1];

            if (is_position_int_invalid(next)) continue;
            if (nodes[next.x][next.y].visited) continue;

            // --- Cost logic ---
            int extra = 1;
            if (costmap[next.x][next.y] == OBSTACLE_COST) {
                extra = 100; // Very costly, but not blocking
            } else if (costmap[next.x][next.y] == MARGIN_COST) {
                extra = 10;  // Intermediate cost
            }

            int new_g = nodes[curr.x][curr.y].g + extra;

            if (new_g < nodes[next.x][next.y].g) {
                nodes[next.x][next.y].g = new_g;
                nodes[next.x][next.y].f = new_g + heuristic(next, start);
                nodes[next.x][next.y].parent = curr;
                open[open_size++] = next;
            }
        }
    }

    // Reconstruct the path (no reversal needed)
    int len = 0;
    position_int_t p = start;

    while (!is_position_int_invalid(p) && !is_position_int_equal(p, goal)) {
        path[len++] = p;
        if (len >= MAX_PATH_LEN) {
            printf("Path exceeds maximum length\n");
            return -1;
        }
        p = nodes[p.x][p.y].parent;
    }

    // Add the goal to the path
    if (len < MAX_PATH_LEN) {
        path[len++] = goal;
    }

    return len;
}

void fill_costmap_square(position_int_t s, position_int_t e, unsigned char cost){
    s.x = (s.x < 0) ? 0 : s.x;
    e.x = (e.x > AS_HEIGHT) ? AS_HEIGHT : e.x;
    s.y = (s.y < 0) ? 0 : s.y;
    e.y = (e.y > AS_WIDTH) ? AS_WIDTH : e.y;
    
    const int width = e.y - s.y;
    if (width <= 0 || e.x <= s.x) return;

    for (int x = s.x; x < e.x; x++) {
        memset(&costmap[x][s.y], cost, (size_t)width);
    }
}

void fill_costmap_sphere(position_int_t center, int radius, unsigned char cost){
    int x0 = center.x;
    int y0 = center.y;

    int r2 = radius * radius;

    int x_min = (x0 - radius < 0) ? 0 : x0 - radius;
    int x_max = (x0 + radius >= AS_HEIGHT) ? AS_HEIGHT - 1 : x0 + radius;

    for (int x = x_min; x <= x_max; x++){
        int dx = x - x0;
        int dy_max = (int)sqrt(r2 - dx*dx);

        int y_start = y0 - dy_max;
        int y_end   = y0 + dy_max;

        if (y_start < 0) y_start = 0;
        if (y_end >= AS_WIDTH) y_end = AS_WIDTH - 1;

        int width = y_end - y_start + 1;
        if (width > 0){
            memset(&costmap[x][y_start], cost, (size_t)width);
        }
    }
}
// c : position centrale en cells
// w, h : largeur/hauteur de l'obstacle en cells
// margin : marge autour de l'obstacle
// traversable : true = A* peut traverser si nécessaire (MARGIN_COST)
//               false = infranchissable (OBSTACLE_COST)
void astar_place_obstacle_with_margin(position_int_t c, int w, int h, int margin, bool traversable, bool square) {
    position_int_t s, e;
    s.x = c.x - h/2;
    e.x = s.x + h;
    s.y = c.y - w/2;
    e.y = s.y + w;

    // Définir la marge autour de l'obstacle
    s.x -= margin; e.x += margin;
    s.y -= margin; e.y += margin;
    unsigned char cost = traversable ? MARGIN_COST : OBSTACLE_COST;
    if (square){
        fill_costmap_square(s, e, cost);
    } else {
        // rayon = moitié du plus grand côté
        int radius = std::max(w, h)/2 + margin;

        fill_costmap_sphere(c, radius, cost);
    }
}

bool is_line_clear(position_int_t a, position_int_t b){
    int dx = abs(b.x-a.x);
    int sx = a.x < b.x ? 1 : -1;
    int dy =-abs(b.y-a.y);
    int sy = a.y < b.y ? 1 : -1;
    int err = dx+dy;
    while(true){
        if (is_position_int_invalid(a)) return false;
        if (costmap[a.x][a.y] >= MARGIN_COST) return false;  // <- ici, obstacle ET marge
        if (is_position_int_equal(a, b)) break;
        int e2 = 2*err;
        if (e2 >= dy) { 
            err += dy; 
            a.x += sx;
        }
        if (e2 <= dx) { 
            err += dx;
            a.y += sy;
        }
    }
    return true;
}

int smooth_path(position_int_t in[], int in_len, position_int_t out[]){
    int out_len = 0, i = 0;
    while (i < in_len) {
        out[out_len++] = in[i];
        int best = i + 1;
        for (int j = in_len - 1; j > i; j--){
            if (is_line_clear(in[i], in[j]))
            {
                best = j;
                break;
            }
        }
        i = best;
    }
    return out_len;
}

int coarse_smooth_path(position_int_t in[], int in_len, position_int_t out[]){
    // Only smooth 1,1 diagonal moves to smooth corners while keeping the path close to the original one, which is important for the robot to be able to follow it more easily
    int out_len = 0;
    for (int i = 0; i < in_len; i++){
        out[out_len++] = in[i];
        if (i < in_len - 2){
            position_int_t a = in[i];
            position_int_t b = in[i+1];
            position_int_t c = in[i+2];
            if (abs(a.x - c.x) == 1 && abs(a.y - c.y) == 1){
                // Zig-zag corner: skip middle point if direct segment is clear
                i++;
            }
        }
    }
    return out_len;
}

unsigned char get_cost(position_int_t p){
    if (is_position_int_invalid(p))
        return 0;
    return costmap[p.x][p.y];
}

double astar_path_length(position_int_t path[], int len){
    double path_length = 0;
    for (int i = 0; i < len - 1; i++){
        int dx = path[i+1].x - path[i].x;
        int dy = path[i+1].y - path[i].y;
        path_length += sqrt((double)(dx*dx + dy*dy));
    }
    return path_length;
}

void print_costmap_with_path(position_int_t *path, int len, position_int_t start, position_int_t goal) {
    LOG_INFO("start : ", start.x, " / ", start.y, "goal : ", goal.x, " / " , goal.y);
    for(int x=0; x<AS_HEIGHT; x++){
        for(int y=0; y<AS_WIDTH; y++){
            if(x == start.x && y == start.y) {printf(" S");continue;}
            if(x == goal.x && y == goal.y) {printf(" G");continue;}

            bool on_path = false;
            for(int i=0; i<len; i++)
                if(path[i].x == x && path[i].y == y) on_path = true;

            if(on_path) printf(" X");
            else if(costmap[x][y] == OBSTACLE_COST) printf(" #");
            else if(costmap[x][y] == MARGIN_COST) printf(" +");
            else printf(" .");
        }
        printf("\n");
    }
}

json astar_get_costmap_json() {
    json costmap_json = json::array();

    for (int y = 0; y < AS_HEIGHT; ++y) {
        for (int x = 0; x < AS_WIDTH; ++x) {
            int cost = costmap[y][x];
            if (cost > 0) {
                costmap_json.push_back({
                    {"x", x},
                    {"y", y},
                    {"cost", cost}
                });
            }
        }
    }

    return costmap_json;
}