#include "utils/debugSound.hpp"

#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "utils/logger.hpp"

namespace {

// A finished player is reaped by this immediately, so no zombie lingers.
void reapPlayer(int) {
    while (waitpid(-1, nullptr, WNOHANG) > 0) {}
}

} // namespace

namespace debugSound {

void play(const std::string& name) {
    const std::string path = "data/sounds/" + name;
    if (access(path.c_str(), R_OK) != 0) {
        LOG_WARNING("Debug sound not found: ", path);
        return;
    }

    // Reap asynchronously: the caller must not wait for the sound to finish.
    static bool handlerInstalled = false;
    if (!handlerInstalled) {
        struct sigaction sa = {};
        sa.sa_handler = reapPlayer;
        sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;
        sigaction(SIGCHLD, &sa, nullptr);
        handlerInstalled = true;
    }

    // `play` (SoX) decodes the file to the default output. A missing `play` only
    // makes the child exit; it never affects this call.
    if (fork() == 0) {
        const int null = open("/dev/null", O_RDWR);
        if (null >= 0) {
            dup2(null, STDIN_FILENO);
            dup2(null, STDOUT_FILENO);
            dup2(null, STDERR_FILENO);
        }
        execlp("play", "play", "-q", path.c_str(), (char*)nullptr);
        _exit(127);
    }
}

} // namespace debugSound
