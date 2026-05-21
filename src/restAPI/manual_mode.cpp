#include "restAPI/manual_mode.h"

bool manual_ctrl = false;
bool (*manual_currentFunc)() = nullptr;

void manual_init()
{
    manual_ctrl = false;
    manual_currentFunc = nullptr;
}

void manual_loop()
{
    if (manual_currentFunc != nullptr){
        // Execute the function as long as it returns false
        if (manual_currentFunc()){
            manual_currentFunc = nullptr;
        }
    }
}

void manual_clearFunc()
{
    manual_currentFunc = nullptr;
}
