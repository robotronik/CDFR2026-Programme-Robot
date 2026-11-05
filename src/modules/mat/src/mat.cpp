#include "mat/mat.hpp"

#include "restAPI/matAPI.hpp"
#include "utils/logger.hpp"

#include <mutex>

namespace {

std::mutex g_matMutex;
MatTableData g_matTableData;

}  // namespace

MatTableData getMatTableData() {
    std::lock_guard<std::mutex> lock(g_matMutex);
    return g_matTableData;
}

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

bool getMapStatus() {
    MatTableData data;
    if (!mat_api::fetchTable(data)) {
        LOG_ERROR("Failed to fetch map status from MAT");
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(g_matMutex);
        g_matTableData = data;
    }
    return true;
}

#else  // DISABLE_MAT

bool StartMat(bool& connectionOk) {
    connectionOk = false;
    return true;
}

void StopMat() {}

bool getMapStatus() {
    return false;
}

#endif  // DISABLE_MAT