#pragma once
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include "utils/logger.hpp"
 
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

/*
* RANSAC developped for take action considering it will be mainly perfect placment blocks
*/
bool findGroupRANSAC2D(
    const std::vector<block_t>& points,
    std::vector<block_t>& bestGroup,
    size_t max_blocks = 4,
    float lineTol = 10.0f,
    float spacing = 50.0f,
    float spacingTol = 10.0f,
    float angleTol = 15.0f  // tolérance orientation
);