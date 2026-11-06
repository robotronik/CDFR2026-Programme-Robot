#pragma once
#include "defs/matTypes.hpp"

// Dernière table reçue du mat, sans appel réseau (le thread de fond de mat_api
// fait les GET). Retourne false si aucune donnée n'est arrivée ou si elle est
// périmée ; `ageMs` reçoit alors l'âge de la dernière donnée (0 si aucune).
bool getMapStatus(MatTableData& data, unsigned long& ageMs);

// Démarre la détection du mat ; non bloquant, `connectionOk` passe à true une
// fois le mat connecté. Les appels suivants ne sont qu'un statut.
bool StartMat(bool& connectionOk);

// Arrête la détection du mat.
void StopMat();
