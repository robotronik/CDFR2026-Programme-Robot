#pragma once

#include <string>
#include <vector>

class Robot;
#include "defs/mainState.hpp"
#include "defs/structs.hpp"

class RestAPI {
    public:
        RestAPI(MainState* currentState, MainState* nextState, Robot* robot);
        ~RestAPI() = default;

        void start();
        void stop();

    private:
        MainState* currentState;
        MainState* nextState;
        Robot* robot;
};