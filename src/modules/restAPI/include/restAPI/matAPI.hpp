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

// Client REST du mat. mat.cpp ne fait pas d'HTTP : il appelle ce module.
// Les requêtes sont faites par un thread de fond, toujours actif : la boucle
// principale ne bloque jamais et ne lit que la dernière table reçue (latest()).
namespace mat_api {

extern const std::string MAT_OBJECTS_PATH;

// Démarre le thread de fond (une seule fois) ; non bloquant. Il interroge
// GET /objects en continu et retente indéfiniment si le mat est absent. Sont
// loggés : l'échec du premier essai au démarrage (erreur), le retour (vert) et
// la perte (erreur) de la connexion ; pas les tentatives suivantes.
// Retourne true si le mat est connecté à cet instant.
bool start();

// Arrête le rafraîchissement et envoie GET /stop. Attend la fin du thread.
void stop();

// Copie la dernière table reçue, sans aucun appel réseau. Retourne false tant
// qu'aucune donnée n'est arrivée ou que la dernière est plus vieille que
// MAT_STALE_MS.
bool latest(MatTableData& data);

// GET /objects bloquant, hors thread de fond (tests / outils).
bool fetchTable(MatTableData& data);

// Vrai si la dernière table reçue n'est pas périmée, sans appel réseau.
bool ready();

}  // namespace mat_api
