#include "defs/matTypes.hpp"

namespace {

double jsonNumber(const json& object, const char* key) {
    const auto it = object.find(key);
    return (it != object.end() && it->is_number()) ? it->get<double>() : 0.0;
}

std::string jsonString(const json& object, const char* key) {
    const auto it = object.find(key);
    return (it != object.end() && it->is_string()) ? it->get<std::string>() : std::string();
}

MatGameElement parseElement(const json& item) {
    MatGameElement element;
    element.id = static_cast<int>(jsonNumber(item, "id"));
    element.label = jsonString(item, "label");
    element.x = jsonNumber(item, "x");
    element.y = jsonNumber(item, "y");
    element.a = jsonNumber(item, "a");
    return element;
}

}  // namespace

MatTableData parseMatTableData(const json& response) {
    MatTableData data;
    if (!response.is_object()) {
        return data;
    }

    const std::string ourColor = jsonString(response, "our_color");
    const std::string opponentColor = jsonString(response, "opponent_color");

    const auto opponents = response.find("opponents");
    if (opponents != response.end() && opponents->is_array()) {
        for (const json& opponent : *opponents) {
            if (opponent.is_object()) {
                data.opponentVisible = true;
                data.opponentX = jsonNumber(opponent, "x");
                data.opponentY = jsonNumber(opponent, "y");
                data.opponentA = jsonNumber(opponent, "a");
                break;
            }
        }
    }

    const auto objects = response.find("objects");
    if (objects != response.end() && objects->is_array()) {
        for (const json& item : *objects) {
            if (!item.is_object()) {
                continue;
            }
            const MatGameElement element = parseElement(item);
            if (!element.label.empty() &&
                (element.label == ourColor || element.label == opponentColor)) {
                continue;  // robot, pas un élément de jeu
            }
            data.elements.push_back(element);
        }
    }

    if (!data.opponentVisible && !opponentColor.empty() &&
        objects != response.end() && objects->is_array()) {
        for (const json& item : *objects) {
            if (item.is_object() && jsonString(item, "label") == opponentColor) {
                data.opponentVisible = true;
                data.opponentX = jsonNumber(item, "x");
                data.opponentY = jsonNumber(item, "y");
                data.opponentA = jsonNumber(item, "a");
                break;
            }
        }
    }

    return data;
}