#include <dirent.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "utils/debugSound.hpp"
#include "utils/logger.hpp"
#include "utils/utils.h"

namespace {

// Number of direct children of this process that are still alive or zombie
// (`play` running, or one that finished but was not reaped).
int childCount() {
    int count = 0;
    DIR* proc = opendir("/proc");
    if (proc == nullptr) return -1;
    while (dirent* entry = readdir(proc)) {
        const int pid = atoi(entry->d_name);
        if (pid <= 0) continue;
        char path[64];
        snprintf(path, sizeof(path), "/proc/%d/stat", pid);
        FILE* f = fopen(path, "r");
        if (f == nullptr) continue;
        char buf[512] = {0};
        const bool read = fgets(buf, sizeof(buf), f) != nullptr;
        fclose(f);
        int ppid = 0;
        char state = 0;
        const char* close = strrchr(buf, ')');
        if (read && close != nullptr && sscanf(close + 1, " %c %d", &state, &ppid) == 2 &&
            ppid == getpid()) {
            ++count;
        }
    }
    closedir(proc);
    return count;
}

} // namespace

// A debug sound must never take the robot down or slow it: a missing file, a
// missing player or no audio output only costs a warning, and play() returns
// immediately instead of waiting for the sound to finish.
bool testDebugSound() {
    bool ok = true;

    // Unknown sample: warns and returns without spawning anything.
    debugSound::play("does-not-exist.wav");

    // play() must not block: starting a ~1 s sample must return far sooner.
    const unsigned long before = _millis();
    debugSound::play("start.wav");
    const unsigned long elapsed = _millis() - before;
    if (elapsed > 250) {
        LOG_ERROR("debugSound::play() blocked the caller for ", elapsed, " ms");
        ok = false;
    }

    // Players must be reaped once they finish, not accumulate as zombies.
    sleep(3); // longer than start.wav
    kill(getpid(), SIGCHLD); // make the pending signal deliver even without a new call
    usleep(20000);
    const int children = childCount();
    if (children > 0) {
        LOG_ERROR("debugSound left ", children, " unreaped player(s)");
        ok = false;
    }
    return ok;
}
