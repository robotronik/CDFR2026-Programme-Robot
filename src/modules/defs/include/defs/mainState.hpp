#pragma once

// Order follows the match flow: Init -> Calibration -> WaitStart -> Run -> Fin,
// then the side states. The raw value is published by the REST API (/get_status,
// /get_global, /get_robot) and mapped in html/*.html.
enum class MainState {
    Init = 0,
    Calibration = 1,
    WaitStart = 2,
    Run = 3,
    Fin = 4,
    Manual = 5,
    Test = 6
};
