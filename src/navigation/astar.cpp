#include "navigation/astar.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include "utils/logger.hpp"

typedef struct {
    int g,f,parent_x,parent_y;
    bool visited;
} Node;

unsigned char costmap[AS_HEIGHT][AS_WIDTH];
Node nodes[AS_HEIGHT][AS_WIDTH];

int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void astar_initialize_costmap(){
    for(int x=0;x<AS_HEIGHT;x++)
        for(int y=0;y<AS_WIDTH;y++)
            costmap[x][y]=FREE_SPACE;
}

void escape_from_obstacle(int *x,int *y){

    if(*x<0||*x>=AS_HEIGHT||*y<0||*y>=AS_WIDTH) return;
    if(costmap[*x][*y]<MARGIN_COST) return;

    int dirs[4][2]={{1,0},{-1,0},{0,1},{0,-1}};
    bool visited[AS_HEIGHT][AS_WIDTH]={0};
    position_t queue[AS_HEIGHT*AS_WIDTH];
    int qs=0,qe=0;
    
    queue[qe++] = (position_t){ static_cast<double>(*x), static_cast<double>(*y) };
    visited[*x][*y]=true;

    while(qs<qe){
        position_t cur=queue[qs++];

        for(int d=0;d<4;d++){
            int nx=cur.x+dirs[d][0];
            int ny=cur.y+dirs[d][1];

            if(nx<0||ny<0||nx>=AS_HEIGHT||ny>=AS_WIDTH) continue;
            if(visited[nx][ny]) continue;

            visited[nx][ny]=true;

            if(costmap[nx][ny]<MARGIN_COST){
                *x=nx; *y=ny;
                return;
            }
            queue[qe++]=(position_t){static_cast<double>(nx),static_cast<double>(ny)};
        }
    }
}

// Récupère le chemin dans points[], retourne la longueur
int reconstruct_path(int sx,int sy,int gx,int gy,position_t *path){

    // Validate start and goal indices to avoid out-of-bounds access on nodes[][]
    if (sx < 0 || sx >= AS_HEIGHT ||
        sy < 0 || sy >= AS_WIDTH  ||
        gx < 0 || gx >= AS_HEIGHT ||
        gy < 0 || gy >= AS_WIDTH) {
        printf("reconstruct_path: invalid indices (sx=%d, sy=%d, gx=%d, gy=%d)\n", sx, sy, gx, gy);
        return 0;
    }
    if(nodes[gx][gy].g==INT_MAX){
        printf("Goal jamais atteint\n");
        return 0;
    }

    int len=0;
    int x=gx,y=gy;

    while(!(x==sx && y==sy)){
        path[len++]=(position_t ){static_cast<double>(x),static_cast<double>(y)};
        int px=nodes[x][y].parent_x;
        int py=nodes[x][y].parent_y;
        x=px; y=py;
        if(len>=500) break;
    }

    path[len++]=(position_t ){static_cast<double>(sx),static_cast<double>(sy)};

    for(int i=0;i<len/2;i++){
        position_t tmp=path[i];
        path[i]=path[len-1-i];
        path[len-1-i]=tmp;
    }

    return len;
}

void astar_pathfind(int *sx,int *sy,int *gx,int *gy){
    escape_from_obstacle(sx,sy);
    escape_from_obstacle(gx,gy);

    int start_x=*sx, start_y=*sy;
    int goal_x=*gx, goal_y=*gy;

    for(int x=0;x<AS_HEIGHT;x++)
        for(int y=0;y<AS_WIDTH;y++){
            nodes[x][y].g=INT_MAX;
            nodes[x][y].f=INT_MAX;
            nodes[x][y].visited=false;
            nodes[x][y].parent_x=-1;
            nodes[x][y].parent_y=-1;
        }

    if(start_x<0||start_x>=AS_HEIGHT||start_y<0||start_y>=AS_WIDTH) return;
    if(goal_x<0||goal_x>=AS_HEIGHT||goal_y<0||goal_y>=AS_WIDTH) return;

    nodes[start_x][start_y].g=0;
    nodes[start_x][start_y].f=heuristic(start_x,start_y,goal_x,goal_y);

    position_int_t open[AS_HEIGHT*AS_WIDTH];
    int open_size=0;
    open[open_size++] = (position_int_t){ static_cast<int>(start_x), static_cast<int>(start_y) };
    int dirs[4][2]={{1,0},{-1,0},{0,1},{0,-1}};

    while(open_size>0){

        int best=0;
        for(int i=1;i<open_size;i++)
            if(nodes[open[i].x][open[i].y].f <
               nodes[open[best].x][open[best].y].f)
                best=i;


        int cx = open[best].x;
        int cy = open[best].y;
        open[best]=open[--open_size];
        if(cx==goal_x && cy==goal_y) return;

        nodes[cx][cy].visited=true;

        for(int d=0;d<4;d++){
            int nx=cx+dirs[d][0];
            int ny=cy+dirs[d][1];

            if(nx<0||ny<0||nx>=AS_HEIGHT||ny>=AS_WIDTH) continue;
            if(costmap[nx][ny]==OBSTACLE_COST) continue;
            if(nodes[nx][ny].visited) continue;

            int extra=(costmap[nx][ny]==MARGIN_COST)?10:1;
            int new_g=nodes[cx][cy].g+extra;

            if(new_g<nodes[nx][ny].g){
                nodes[nx][ny].g=new_g;
                nodes[nx][ny].f=new_g+heuristic(nx,ny,goal_x,goal_y);
                nodes[nx][ny].parent_x=cx;
                nodes[nx][ny].parent_y=cy;
                open[open_size++]=(position_int_t){nx,ny};
            }
        }
    }
}


void place_obstacle_with_margin(int x0_mm,int y0_mm,int w_mm,int h_mm,int RayonRobot){
    int x0=(x0_mm+1000)/SCALE;
    int y0=(y0_mm+1500)/SCALE;
    int w=w_mm/SCALE, h=h_mm/SCALE;
    int margin=RayonRobot/SCALE;

    int x_start=x0-h/2;
    int x_end=x_start+h;
    int y_start=y0-w/2;
    int y_end=y_start+w;

    for(int x=x_start;x<x_end;x++)
        for(int y=y_start;y<y_end;y++)
            if(x>=0 && x<AS_HEIGHT && y>=0 && y<AS_WIDTH)
                costmap[x][y]=OBSTACLE_COST;

    x_start-=margin; x_end+=margin;
    y_start-=margin; y_end+=margin;

    for(int x=x_start;x<x_end;x++)
        for(int y=y_start;y<y_end;y++)
            if(x>=0 && x<AS_HEIGHT && y>=0 && y<AS_WIDTH)
                if(costmap[x][y]!=OBSTACLE_COST)
                    costmap[x][y]=MARGIN_COST;
}

int line_max_cost(position_t p1, position_t p2) {
    int x0 = p1.x, y0 = p1.y;
    int x1 = p2.x, y1 = p2.y;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;
    int max_cost = 0;

    while (true) {
        if (x0 < 0 || x0 >= AS_HEIGHT || y0 < 0 || y0 >= AS_WIDTH)
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

bool line_clear(position_t a, position_t b){
    int x0=a.x, y0=a.y, x1=b.x, y1=b.y;
    int dx=abs(x1-x0), sx=x0<x1?1:-1;
    int dy=-abs(y1-y0), sy=y0<y1?1:-1;
    int err=dx+dy;
    while(true){
        if(x0<0 || x0>=AS_HEIGHT || y0<0 || y0>=AS_WIDTH) return false;
        if(costmap[x0][y0] >= MARGIN_COST) return false;  // <- ici, obstacle ET marge
        if(x0==x1 && y0==y1) break;
        int e2=2*err;
        if(e2>=dy){ err+=dy; x0+=sx; }
        if(e2<=dx){ err+=dx; y0+=sy; }
    }
    return true;
}

int smooth_path(position_t *in,int in_len,position_t *out){
    int out_len=0,i=0;
    while(i<in_len){
        out[out_len++]=in[i];
        int best=i+1;
        for(int j=in_len-1;j>i;j--)
            if(line_clear(in[i],in[j])){ best=j; break; }
        i=best;
    }
    return out_len;
}

void print_costmap_with_path(position_t *path, int len, position_int_t start, position_int_t goal) {
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