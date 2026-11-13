#include "utils/debugSound.hpp"

#include <fcntl.h>
#include <sched.h>
#include <sys/wait.h>
#include <unistd.h>

#include "utils/logger.hpp"

namespace debugSound {

void play(const std::string& name) {
    const std::string path = "data/sounds/" + name;
    if (access(path.c_str(), R_OK) != 0) {
        LOG_WARNING("Debug sound not found: ", path);
        return;
    }

    // Double fork: init adopts the player and reaps it, so we never wait on it.
    const pid_t middle = fork();
    if (middle == 0) {
        if (fork() == 0) {
            // Don't inherit the control loop's real-time priority.
            const sched_param normal = {};
            sched_setscheduler(0, SCHED_OTHER, &normal);
            const int null = open("/dev/null", O_RDWR);
            dup2(null, STDIN_FILENO);
            dup2(null, STDOUT_FILENO);
            dup2(null, STDERR_FILENO);
            execlp("play", "play", "-q", path.c_str(), (char*)nullptr);
        }
        _exit(0);
    }
    if (middle > 0) waitpid(middle, nullptr, 0);
}

} // namespace debugSound
