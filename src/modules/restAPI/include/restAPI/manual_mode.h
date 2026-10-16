#ifndef MANUAL_MODE_H
#define MANUAL_MODE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern bool manual_ctrl;
extern bool (*manual_currentFunc)();

void manual_init();
void manual_loop();
void manual_clearFunc();

#ifdef __cplusplus
}
#endif

#endif // MANUAL_MODE_H
