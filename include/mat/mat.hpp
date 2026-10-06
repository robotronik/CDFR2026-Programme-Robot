#pragma once
#include "utils/json.hpp" // For handling JSON

#include <string>
#include <vector>

using json = nlohmann::json;

#ifndef MAT_HOST
#define MAT_HOST "0.0.0.0"
#endif
#ifndef MAT_PORT
#define MAT_PORT 5000
#endif

extern const std::string MAT_URL;

// Un élément de jeu vu par le mat (repère table : mm et degrés).
struct MatGameElement {
    int id = 0;
    std::string label;
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;
};

// Dernier état de la table reçu du mat (rempli par getMapStatus()).
struct MatTableData {
    bool opponentVisible = false;
    double opponentX = 0.0;
    double opponentY = 0.0;
    double opponentA = 0.0;
    std::vector<MatGameElement> elements;
};

// Convertit une réponse de /fleet/live en données exploitables (testable sans réseau).
MatTableData parseMatTableData(const json& response);

// Dernier état reçu du mat (lu par TableState::updateMapStatus()).
MatTableData getMatTableData();

bool getMapStatus();
bool StartMat(bool& connectionOk);
void StopMat();