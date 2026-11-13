# Debug sounds

Short sounds that tell you what the program is doing: startup, end of match,
shutdown, and any call you add:

```cpp
#include "utils/debugSound.hpp"

debugSound::play("trouble.wav");   // plays data/sounds/trouble.wav
```

`play()` starts SoX's `play` command in the background and returns at once. A
missing file, player or speaker just means silence; it never blocks or crashes
the robot.

Sounds live in [`data/sounds/`](data/sounds) (any format SoX reads: `wav`,
`mp3`, `ogg`, ...). Already used:

| Sound | When |
|---|---|
| `start.wav` | end of the init sequence |
| `end.wav` | match over |
| `shutdown.wav` | final stop |

`bip`, `boup`, `bipboup`, `ok` and `trouble` are spare samples for your own calls.

## On the robot (Pi 5)

The Pi 5 has no audio jack, so use a **bus-powered USB speaker** (e.g. Adafruit
Mini USB Stereo Speaker, PID 3369). The USB ports share 600 mA on a 3 A supply:
avoid speakers with a big amp, they cause brown-outs.

1. Install SoX once: `sudo apt install sox libsox-fmt-all`.
2. Plug the speaker in **before deploying**: the installer sets the service to
   use the first USB sound card (the Pi's default output is HDMI).
3. Test with `sudo systemctl restart programCDFR`. Running `sudo ./programCDFR`
   by hand doesn't get that setting.

## On a laptop

| Command | Setup |
|---|---|
| `./build.sh run-docker` | nothing |
| `./build.sh run` | install SoX on the host (`sudo pacman -S sox` or `sudo apt install sox`) |
| `build.bat run` (Windows) | Docker Desktop with WSL 2 (untested) |

The program runs as root and uses your PipeWire server. A plain PulseAudio
server refuses root, so it stays silent there.

## Silent?

Run `play -q data/sounds/start.wav` in the same place (the host, the Pi, or
`./build.sh shell`) to see the error.

- `play: command not found` in Docker: your image is older than SoX, rebuild it
  with `./build.sh images`.
- On the Pi: `aplay -l` should list the speaker, and `systemctl cat programCDFR`
  should show its `AUDIODEV`.
