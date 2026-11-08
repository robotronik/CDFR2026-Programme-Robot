#pragma once

#include <string>

// Debug sounds: audible markers for program paths and decisions, played on the
// robot's speakers or on a laptop running the Docker image (see AUDIO.md).
//
// Files live in data/sounds/ (anything SoX decodes: mp3, ogg, flac, wav, ...).
namespace debugSound {

// Plays data/sounds/<name> without waiting for it to finish, so it never blocks
// the control loop. A missing file or a missing player only logs/does nothing.
void play(const std::string& name);

} // namespace debugSound
