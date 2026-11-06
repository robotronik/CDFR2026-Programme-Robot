#include "mat/mat.hpp"

#include "restAPI/matAPI.hpp"
#include "utils/logger.hpp"

#ifndef DISABLE_MAT

bool StartMat(bool& connectionOk) {
    static unsigned long startTime = 0;

    if (mat_api::start(connectionOk)) {
        startTime = 0;
        return true;
    }

    if (startTime == 0) {
        startTime = _millis();
    } else if (_millis() - startTime > 5000) {
        LOG_ERROR("MAT failed to start within timeout");
        startTime = 0;
        connectionOk = false;
        return true;
    }
    LOG_EXTENDED_DEBUG("Waiting for MAT to start...");
    return false;
}

void StopMat() {
    mat_api::stop();
}

bool getMapStatus(MatTableData& data, unsigned long& ageMs) {
    return mat_api::latest(data, ageMs);
}

#else  // DISABLE_MAT

bool StartMat(bool& connectionOk) {
    connectionOk = false;
    return true;
}

void StopMat() {}

bool getMapStatus(MatTableData&, unsigned long& ageMs) {
    ageMs = 0;
    return false;
}

#endif  // DISABLE_MAT
