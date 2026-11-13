# Debug sounds

Short sounds that mark program paths and decisions in real time: startup, end of
match, shutdown, and any `debugSound::play("...")` you add.

They play through the external `play` command (SoX), asynchronously: the call
returns immediately, so a sound never blocks the control loop. A missing file or
player does nothing, and a missing audio output just means silence — it can
neither stall nor crash the robot.

## Hardware: the Pi 5 has no audio jack

The Raspberry Pi 5 removed the 3.5 mm AV jack that the Pi 4 and earlier had; it
outputs audio only over HDMI, USB and I2S. So speakers must be **USB**: plug them
into any Pi 5 USB port and they become the default output, no driver and no GPIO.

Mind the power budget: the Pi 5 leaves **600 mA total** to its USB ports on a 3 A
supply (1.6 A with the official 5 A supply), and that budget is shared with the
fan header. Pick a **bus-powered** speaker (no battery, no separate adapter) that
stays well under it:

| Speaker | Notes |
|---|---|
| Adafruit Mini USB Stereo Speaker (PID 3369) | USB only, bus-powered, 2 × 2 W, 84 × 43 × 32 mm, ~74 g. The safest default. |
| Generic "Raspberry Pi USB free-drive speaker" | Small stick-style USB speaker sold for the Pi; check it is bus-powered. |

Avoid speakers with their own loud amp or a 3.5 mm-only input: a 2 × 5 W+ amp
will exceed the USB budget and cause brown-outs. Keep the volume moderate — beeps
only need to be audible.

## Using it

```cpp
#include "utils/debugSound.hpp"

debugSound::play("trouble.wav");   // data/sounds/trouble.wav
```

Drop any file SoX can decode (`mp3`, `ogg`, `flac`, `wav`, ...) in
[`data/sounds/`](data/sounds) and pass its file name.

Already hooked:

| Sound | When |
|---|---|
| `start.wav` | end of the init sequence (`StartSequence`) |
| `end.wav` | match over (`FIN`: 100 s elapsed, strategy finished, or button) |
| `shutdown.wav` | final stop (`EndSequence`) |

`bip`, `boup`, `bipboup`, `ok` and `trouble` are general-purpose samples for your
own calls.

## Where is it heard?

`play` uses the default audio output of the machine it runs on (in Docker, the
container's):

| Where | Hears it | What to do |
|---|---|---|
| Raspberry Pi | ✅ USB speaker | install SoX on the Pi once (`sudo apt install sox libsox-fmt-all`, see [README](README.md#runtime-dependencies-on-the-pi)); plug the speaker in a USB port |
| Laptop, Linux, `./build.sh run-docker` | ✅ laptop speakers | nothing: `build.sh` shares `$XDG_RUNTIME_DIR/pulse` with the container when a PulseAudio/PipeWire server runs |
| Laptop, Linux, `./build.sh run` | ✅ laptop speakers | nothing (the program runs on the host) |
| Laptop, Windows, `build.bat run` | ✅ laptop speakers (WSLg) | nothing: `build.ps1` mounts `/mnt/wslg/PulseServer` |

On Linux the audio server's socket is shared read-only; no audio device is handed
to the container. If the server is not at the default `/run/user/<uid>/pulse`
location, or the host uses plain ALSA, `play` falls back to ALSA's `default`
device.

## Checking the output

Just start the program: `./build.sh run-docker` plays the startup sound on launch
(`sudo ./programCDFR` on the Pi).

If nothing is heard, play a sample directly in the same environment (the host, or
`./build.sh shell` for a container) to see SoX's error:

```bash
play -q data/sounds/start.wav
```

On the Pi, `aplay -l` lists the detected output (the USB speaker should appear).
