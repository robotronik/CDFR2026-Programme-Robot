#pragma once
#include "defs/matTypes.hpp"

// Dernière table reçue du mat, sans appel réseau (le thread de fond de mat_api
// fait les GET). Retourne false si aucune donnée n'est arrivée ou si elle est
// périmée.
bool getMapStatus(MatTableData& data);

// Démarre (une seule fois) le thread de fond du mat ; non bloquant. Retourne
// true si le mat est connecté à cet instant ; sinon il retente en continu en
// tâche de fond.
bool StartMat();

// Arrête le thread du mat (et envoie GET /stop).
void StopMat();
