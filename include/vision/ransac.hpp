#pragma once
#include "vision/geom.hpp"

/*
* RANSAC developped for take action considering it will be mainly perfect placment blocks
*/
bool findGroupRANSAC2D(
    const std::vector<block_t>& points,
    std::vector<block_t>& bestGroup,
    size_t max_blocks = 4,
    float lineTol = 10.0f,
    float spacing = 50.0f,
    float spacingTol = 15.0f,
    float angleTol = 15.0f  // tolérance orientation
);

/*
* RANSAC developped for steal action considering it will be a shinanigan
*/
bool findGroupStealRANSAC2D(
    const std::vector<block_t>& points,
    std::vector<block_t>& bestGroup,
    size_t max_blocks = 4,
    float lineTol = 40.0f,
    float spacing = 50.0f,
    float spacingTol = 100.0f,
    float angleTol = 45.0f  // tolérance orientation
);