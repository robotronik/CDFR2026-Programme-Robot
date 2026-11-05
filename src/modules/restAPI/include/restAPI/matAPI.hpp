#pragma once
#include "defs/matTypes.hpp"

#include <string>

#ifndef MAT_HOST
#define MAT_HOST "192.168.0.135"
#endif
#ifndef MAT_PORT
#define MAT_PORT 5000
#endif

// Client REST du mat. mat.cpp ne fait plus d'HTTP : il appelle ce module.
namespace mat_api {

extern const std::string MAT_OBJECTS_PATH;

// GET /start : démarre la détection. La requête n'est faite qu'une fois, les
// appels suivants ne sont qu'une lecture du statut (aucun appel réseau).
bool start(bool& connectionOk);

// GET /stop : arrête la détection.
void stop();

// GET /objects : récupère l'adversaire et les objets de jeu.
bool fetchTable(MatTableData& data);

// Dernier statut connu du mat, sans appel réseau.
bool ready();

}  // namespace mat_api