#include "mat/mat.hpp"

#include "restAPI/matAPI.hpp"

bool StartMat() {
    return mat_api::start();
}

void StopMat() {
    mat_api::stop();
}

bool getMapStatus(MatTableData& data) {
    return mat_api::latest(data);
}
