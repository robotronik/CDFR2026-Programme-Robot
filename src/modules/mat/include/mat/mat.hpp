#pragma once
#include "defs/matTypes.hpp"

#include <string>

// Field mat client: polls GET /objects on a background thread and keeps the last
// table. The main loop never blocks; it only reads the latest snapshot.
class Mat {
    public:
        static const std::string MAT_OBJECTS_PATH;

        // Starts the background thread (once); non-blocking. Returns true if the
        // mat is reachable right now.
        bool start();
        // Stops the background thread and sends GET /stop.
        void stop();
        // Copies the latest table received, without any network call.
        bool latest(MatTableData& data);
        // Blocking GET /objects (tests / tools).
        bool fetchTable(MatTableData& data);
        // True if the last table received is fresh, without any network call.
        bool ready();
};
