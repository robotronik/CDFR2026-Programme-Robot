#ifndef UTILS_H
#define UTILS_H

#include <time.h>
#include <cstdlib>
#include <string>
#include <fstream>

inline unsigned long _millis() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // Utilisation de CLOCK_MONOTONIC pour éviter les changements d'horloge
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000); // Conversion en millisecondes
}

inline std::string currentTimeFormatted() {
    time_t temps;
    struct tm date;
    char buffer[32];
    time(&temps);
    date = *localtime(&temps);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &date);
    return std::string(buffer);
}

inline bool isWifiConnected()
{
    std::ifstream file("/proc/net/wireless");
    std::string line;

    if (file.is_open())
    {
        while (std::getline(file, line))
        {
            if (line.find("wlan0") != std::string::npos)
            {
                return true;
            }
        }
        file.close();
    }
    return false;
}

inline void setProgramPriority(int val = 99) // Max priority
{
    struct sched_param param;
    param.sched_priority = 99;  // Highest priority for SCHED_FIFO

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
        throw std::runtime_error("sched_setscheduler failed. Make sure the programs runs in SUDO");
    }
    printf("Program priority set to %d\n", val);
}

#endif //UTILS_H