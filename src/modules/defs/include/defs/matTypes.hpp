#pragma once
#include "utils/json.hpp"

#include <string>
#include <vector>

using json = nlohmann::json;

struct MatGameElement {
    int id = 0;
    std::string label;
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;
};

struct MatTableData {
    bool opponentVisible = false;
    double opponentX = 0.0;
    double opponentY = 0.0;
    double opponentA = 0.0;
    std::vector<MatGameElement> elements;
};

// Convertit une réponse GET /objects du mat. Les tags de couleur sont des
// robots : ils n'alimentent pas `elements`.
MatTableData parseMatTableData(const json& response);