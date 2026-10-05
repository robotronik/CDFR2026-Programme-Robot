#include "mat/mat.hpp"

// Vérifie la lecture d'une réponse /fleet/live du mat : position de l'adversaire
// et objets de jeu, y compris les champs manquants ou nuls.
bool test_mat_parse() {
    const json payload = json::parse(R"JSON({
        "our_color": "blue",
        "opponent_color": "yellow",
        "objects": [
            {"id": 13, "label": "element", "x": 705.2, "y": -410.9, "a": 45.3},
            {"id": 13, "label": "element", "x": 710.1, "y": -398.2, "a": -44.6}
        ],
        "opponents": [{"id": 6, "x": -250.0, "y": -400.0, "a": -75.0}],
        "robots": [{"key": "main", "x": 300.0, "y": 250.0, "a": 30.0}]
    })JSON");

    const MatTableData data = parseMatTableData(payload);
    if (!data.opponentVisible) return false;
    if (data.opponentX != -250.0 || data.opponentY != -400.0 || data.opponentA != -75.0) return false;
    if (data.elements.size() != 2) return false;
    if (data.elements[0].id != 13 || data.elements[0].label != "element") return false;
    if (data.elements[0].x != 705.2 || data.elements[1].a != -44.6) return false;

    // Réponse vide : aucune donnée, aucun plantage.
    const MatTableData empty = parseMatTableData(json::object());
    if (empty.opponentVisible || !empty.elements.empty()) return false;

    // Champs manquants ou nuls : valeurs par défaut.
    const MatTableData partial = parseMatTableData(json::parse(
        R"({"opponents":[{"id":6,"x":null,"y":1.5}],"objects":[{"id":null,"label":null,"x":3}]})"));
    if (!partial.opponentVisible || partial.opponentX != 0.0 || partial.opponentY != 1.5) return false;
    if (partial.elements.size() != 1 || partial.elements[0].id != 0) return false;
    if (!partial.elements[0].label.empty() || partial.elements[0].x != 3.0) return false;

    return true;
}

bool test_start_mat(){
    // Is it useful ?
    bool sucess;
    while(!StartMat(sucess));
    return sucess;
}
