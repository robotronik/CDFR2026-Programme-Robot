#pragma once

#include "defs/mainState.hpp"

class Robot;

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