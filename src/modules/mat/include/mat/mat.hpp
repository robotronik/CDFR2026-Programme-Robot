#pragma once
#include "defs/matTypes.hpp"

// Dernier état reçu du mat (lu par TableState::updateMapStatus()).
MatTableData getMatTableData();

// Interroge le mat (GET /objects via le module REST) et mémorise l'état reçu.
bool getMapStatus();

// Démarre la détection du mat ; les appels suivants ne sont qu'un statut.
bool StartMat(bool& connectionOk);

// Arrête la détection du mat (GET /stop via le module REST).
void StopMat();