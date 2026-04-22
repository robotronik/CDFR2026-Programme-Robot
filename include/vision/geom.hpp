#pragma once
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include "utils/logger.hpp"
#include <tuple>

// --- Paramètres de collision ---
const float ROBOT_RADIUS = 50.0f;
const float BLOCK_RADIUS = 80.0f; 
const float CORRIDOR_THRESHOLD = ROBOT_RADIUS + BLOCK_RADIUS; 
const float TARGET_MARGIN = 30.0f; 

typedef struct {
    double x = 0;
    double y = 0;
    double a = 0;
    bool color = false; //true for Blue false for Yellow
}block_t;

struct Line {
    float x, y;
    float dx, dy;
};

// normalisation angle [-180, 180]
inline float angleDiff(float a, float b);

// distance point-droite 2D
inline float pointLineDistance(const block_t& p, const Line& l);

// projection scalaire
inline float project(const block_t& p, const Line& l);

// Produit vectoriel 2D (déterminant) pour savoir de quel côté on est
inline float cross2d(float x1, float y1, float x2, float y2);

//détermine si un angle entre 2 blocks est inférieur à la tolérance
bool acceptableAngle(const block_t* b1, const block_t* b2, float angleTol);

// Vérifie si un point est trop proche du segment [O, T] (O étant le robot)
bool checkSegment(float px, float py, float tx, float ty);

//Détermine la position optimale du centre du poussoir 
block_t placePoussoir(const std::vector<const block_t*>& choosen, const std::vector<block_t>& points);
block_t interfacePlacePoussoir(const std::vector<std::pair<float, const block_t*>>& choosen, const std::vector<block_t>& points);

bool blockInFrontInterface(const std::vector<std::tuple<float, const block_t*, bool>>& choosen, const std::vector<block_t>& points);
bool blockInFrontInterface(const std::vector<std::pair<float, const block_t*>>& choosen, const std::vector<block_t>& points);
/*
* return true s'il y a des blocks entre le robot et les blocks qu'il souhaite prendre false sinon
*/
bool blockInFront(const std::vector<const block_t*>& choosen, const std::vector<block_t>& points);