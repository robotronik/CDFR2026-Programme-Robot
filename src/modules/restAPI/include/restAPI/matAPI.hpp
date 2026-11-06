#pragma once
#include "defs/matTypes.hpp"

#include <string>

#ifndef MAT_HOST
#define MAT_HOST "192.168.0.135"
#endif
#ifndef MAT_PORT
#define MAT_PORT 5000
#endif

// Période de rafraîchissement de la table par le thread de fond.
#ifndef MAT_POLL_PERIOD_MS
#define MAT_POLL_PERIOD_MS 100
#endif
// Au-delà de cet âge, la dernière table reçue est considérée comme périmée.
#ifndef MAT_STALE_MS
#define MAT_STALE_MS 120
#endif

// Client REST du mat. mat.cpp ne fait plus d'HTTP : il appelle ce module.
// Les requêtes sont faites par un thread de fond : la boucle principale ne
// bloque jamais et ne lit que la dernière table reçue (latest()).
namespace mat_api {

extern const std::string MAT_OBJECTS_PATH;

// Demande le démarrage de la détection. Non bloquant : le thread de fond envoie
// GET /start puis rafraîchit la table en continu. `connectionOk` passe à true
// dès que le mat a répondu ; les appels suivants ne font que relire le statut.
bool start(bool& connectionOk);

// Arrête le rafraîchissement et envoie GET /stop. Attend la fin du thread.
void stop();

// Copie la dernière table reçue, sans aucun appel réseau. Retourne false tant
// qu'aucune donnée n'est arrivée ou que la dernière est plus vieille que
// MAT_STALE_MS ; `ageMs` reçoit alors l'âge de la dernière donnée (0 si aucune).
bool latest(MatTableData& data, unsigned long& ageMs);

// GET /objects bloquant, hors thread de fond (tests / outils).
bool fetchTable(MatTableData& data);

// Dernier statut connu du mat, sans appel réseau.
bool ready();

}  // namespace mat_api
