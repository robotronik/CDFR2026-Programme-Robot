# 🤖 CDFR-Programme-Robot

Welcome to the **CDFR-Programme-Robot** project!

This project aims to develop a program to control a robot for the CDFR  event.

## 📖 Description

This program enables the robot to perform various tasks such as navigation, data collection, and more using a modular and extensible design.

## 🚀 Features

- **Navigation**: The robot can move through its environment using dedicated algorithms.
- **Vision**: Native C++ ArUco marker detection and feature-based localisation using
  OpenCV, running in-process (no external Python service). Camera capture uses
  libcamera on the Raspberry Pi 5 and OpenCV/V4L2 elsewhere.
- **Data Collection**: The robot gathers and stores data from onboard sensors.
- **Communication**: The program supports communication with other systems or devices.

## 🔧 Prerequisites

- **Docker** (daemon running) — all compilation happens inside containers, so no
  compiler, CMake, or library needs to be installed on your machine.
- SSH access to the robot, only if you want to deploy.

Nothing else is needed for `build`, `test`, `deploy`, `shell` or `run-docker`.
Only `run`, which executes the binary directly on the host instead of inside the
image, needs the runtime libraries listed in
[Running on the host](#-running-on-the-host).

## 📥 Installation

1. **Do not clone this repository by itself!**  
   Instead, clone the main CDFR repository with the `--recursive` flag so the
   `rplidar_sdk` submodule is checked out:

   ```bash
   git clone git@github.com:robotronik/CDFR.git --recursive
   ```

   If you already cloned it without `--recursive`, initialize the submodule:

   ```bash
   git submodule update --init --recursive
   ```

2. Navigate to the project directory:

   ```bash
   cd informatique/CDFR2026-Programme-Robot/
   ```

The `drive_interface.h` / `protocol.h` headers are fetched automatically into the
build images, and OpenCV, SQLite and (on ARM) libcamera are installed there too,
so nothing needs to be installed on the host. The camera calibration files used
at runtime live in [`data/`](data).

## 💻 Compilation

Everything runs inside Docker; the host compiler is never used.

```bash
./build.sh build          # Build both x86_64 and arm64
./build.sh build x86_64   # Build a single target
./build.sh build arm64
./build.sh run            # Build x86_64 and run it on the host (needs sudo + OpenCV 4.6)
./build.sh run-docker     # Build x86_64 and run it inside the Docker image (no host deps)
./build.sh test           # Build x86_64 and run the CTest suite
./build.sh deploy         # Build arm64 and deploy it to the robot
./build.sh shell          # Interactive shell in the x86_64 image
./build.sh shell arm64    # Interactive shell in the arm64 image
./build.sh images         # (Re)build the Docker images
./build.sh clean          # Remove the build/ directory
```

`./build.sh run` executes the x86_64 binary from `build/x86_64` (so it finds its
`html/` and `data/` assets) and needs `sudo` because the REST server binds port 80,
on top of the host libraries listed below.

`./build.sh run-docker` runs the very same binary inside the `cdfr-builder-x86_64`
image instead. It needs neither `sudo` nor any host library, and it is the
portable way to run the program when the host's OpenCV version differs from the
one used to build (the binary is linked against OpenCV 4.6 and will fail with
`libopencv_aruco.so.406: cannot open shared object file` on a host that ships a
different version). Internally the container runs as root — needed so that
`--cap-add=SYS_NICE` can raise the program's real-time priority, the same thing
`sudo` does for `run` — and binds port 80 through `--network host`. As a result
the files it writes (the `log/` directory) are owned by root on the host.

Local x86_64 builds have no hardware, so they use the emulated I2C, disable the
lidar, and run the API in test mode; the **MAT is disabled** as well (it is the
robot's vision server) — the ARM build keeps it enabled. Override with
`-DCDFR_ENABLE_MAT=ON` when needed.

### 🏃 Running on the host

`./build.sh run` executes the binary outside Docker, so the host must provide the
same shared libraries the binary was linked against. Those come from the image's
OpenCV 4.6 (Ubuntu 24.04 packaging), plus `sudo`/`CAP_SYS_NICE` for the real-time
scheduler and port 80.

On **Ubuntu 24.04**:

```bash
sudo apt install libopencv-contrib406t64 libopencv-videoio406t64
```

`libopencv-contrib406t64` brings the contrib module set (`libopencv_aruco.so.406`)
and its core dependencies; `libopencv-videoio406t64` provides the camera/video
I/O module. Both are needed — a plain `libopencv-dev` install of a *different*
OpenCV version is not enough, since the loader matches the exact `406` ABI.

On a host whose OpenCV is a different version (for example 25.04 ships OpenCV
4.10 as `libopencv_*410`), these libraries cannot be installed from the
distribution repositories. Use `./build.sh run-docker` there instead of installing
OpenCV by hand.

Artifacts are written to `build/x86_64/programCDFR` and
`build/arm64/programCDFR`. Each build directory also contains the runtime bundle
shipped alongside the executable (`html/`, `data/`, `tests/lidar` and
`autoRunInstaller.sh`); CI zips these into the `programCDFR-<arch>` artifacts.

- The workspace is mounted at its own path, so build outputs appear directly on
  the host and file ownership is preserved.
- The two images are defined in [`docker/Dockerfile.x86_64`](docker/Dockerfile.x86_64)
  and [`docker/Dockerfile.arm64`](docker/Dockerfile.arm64); rebuild them with
  `./build.sh images` after changing their contents.
- VS Code / CLion Dev Container support is available via
  [`.devcontainer/devcontainer.json`](.devcontainer/devcontainer.json).

### 🪟 Windows (Docker Desktop)

On Windows, use `build.bat` (a thin launcher for `build.ps1`). It drives the same
Docker images as `build.sh`, so no compiler, CMake or library is needed on the
host — only Docker Desktop, which must be running.

```bat
build.bat build          REM Build both x86_64 and arm64
build.bat build arm64
build.bat run            REM Build x86_64 and run it locally (REST API on http://localhost)
build.bat test           REM Build x86_64 and run the CTest suite
build.bat deploy         REM Build arm64 and deploy it to the robot
build.bat shell          REM Interactive shell in the x86_64 image
build.bat images         REM (Re)build the Docker images
build.bat clean          REM Remove the build\ directory
```

Windows-specific notes:

- The repository is mounted at `/work` inside the container (Windows paths cannot
  be reused as Linux paths as in `build.sh`); build outputs still appear on the
  host under `build\<arch>`.
- `build.bat run` publishes container port 80 on `http://localhost`. Override the
  host port with the `CDFR_RUN_PORT` environment variable
  (e.g. `set CDFR_RUN_PORT=8080`).
- `build.bat deploy` authenticates with the SSH keys in `%USERPROFILE%\.ssh`,
  mounted read-only into the container. Set them up for the robot first (the
  Windows OpenSSH client ships with `ssh-keygen`/`ssh`; `ssh-copy-id` is not
  available, so append your public key to the robot's `~/.ssh/authorized_keys`).
  A passphrase-protected key needs an SSH agent (`ssh-agent` + `ssh-add`).
- The x86_64 build disables `compile_commands.json` export on Windows because the
  symlink it creates cannot be written reliably on a Windows bind mount.

## 🛠️ Compilation for Raspberry Pi

The ARM binary is cross-compiled by `./build.sh build arm64`; no cross-toolchain or
sysroot is needed on the host. It targets **Raspberry Pi OS Trixie (64-bit)**: the
build image is Debian 13, so the binary is linked against the same libraries the Pi
ships — GCC 14 / glibc 2.41, OpenCV 4.10 (`libopencv_*.so.410`) and the Raspberry Pi
libcamera fork (`libcamera.so.0.7`).

### Runtime dependencies on the Pi

The deployed bundle only contains the executable (`programCDFR`), `html/`, `data/`
and `autoRunInstaller.sh`; the shared libraries come from the OS. On a 64-bit
Raspberry Pi OS Trixie, install them once with:

```bash
sudo apt update
sudo apt install -y \
  libsqlite3-0 \
  libopencv-objdetect410 libopencv-imgcodecs410 libopencv-calib3d410 \
  libopencv-features2d410 libopencv-imgproc410 libopencv-core410 \
  libcamera0.7 libcamera-ipa libpisp1
```

ArUco lives in the core OpenCV `objdetect` module from 4.7 onwards, hence
`libopencv-objdetect410` (which pulls the remaining OpenCV libraries);
`libcamera0.7` (from `archive.raspberrypi.com`) is the camera stack the binary was
linked against and pulls `libcamera-ipa`/`libpisp1`.

Then enable the I2C bus and the UART (used by the actuators and the lidar) via:

```bash
sudo raspi-config
```

(The ARM binary is `aarch64` and cannot run on a 32-bit OS.)

To deploy it to your Raspberry Pi, first set up SSH key authentication. To copy your SSH key to the Raspberry Pi (replace `pi@192.168.1.47` with your Raspberry Pi’s address):

```bash
ssh-copy-id pi@192.168.1.47
```

Then compile and deploy with:

```bash
./build.sh deploy
```

## 🔍 Service Monitoring and Restart

To view the service logs:

```bash
journalctl -b -u programCDFR --output=cat
journalctl -u programCDFR -f --output=cat
```

To list the active services:

```bash
systemctl list-units --type=service
```

To reload the service configuration and restart the program:

```bash
sudo systemctl daemon-reload
sudo systemctl restart programCDFR
```
To ensure a backup of the logs

```bash
sudo nano /etc/systemd/journald.conf
```
and add
```bash
[Journal]
Storage=persistent
SyncIntervalSec=2s
```


## 🐞 Debugging on Raspberry Pi with VS Code

1. Connect your PC to the same Wi-Fi network as the Raspberry Pi.
2. Update the IP address in `launch.json` and `task.json` to match your robot's address.
3. Press F5 in VS Code to start remote debugging, set breakpoints, and utilize VS Code's debugging tools.

## 🌐 Website Access (REST API)

Ensure that both the robot and the program are running and that you are on the same local network. Then, open your browser and go to:

```url
http://raspitronik.local
```

## 📺 Touchscreen on the Robot

To set up the touchscreen kiosk mode on the Raspberry Pi, first disable NTP and set the date to avoid SSL issues:
```bash
sudo timedatectl set-ntp false
sudo timedatectl set-time '2025-12-02 19:40:00'
sudo apt-get update
sudo apt-get upgrade -y
```

Then, install Xorg, Openbox, and Chromium if not already installed:

```bash
sudo apt install libcamera-apps
sudo apt-get install xorg openbox chromium-browser
sudo apt install xorg openbox -y
export DISPLAY=:0
sudo startx /usr/bin/chromium-browser --noerrdialogs --kiosk http:localhost/robot --incognito --disable-extensions --no-sandbox
```

Alternatively, use:

```bash
/usr/bin/chromium-browser --kiosk http:localhost/robot --incognito --disable-extensions
```

For configuring a long display, edit the configuration file:

```bash
sudo nano /boot/firmware/config.txt
```

And add the following line:

```bash
# Automatically load overlays for detected DSI displays
display_auto_detect=1

# Automatically load initramfs files, if found
auto_initramfs=1

# Enable DRM VC4 V3D driver
dtoverlay=vc4-kms-v3d
dtoverlay=vc4-kms-dsi-waveshare-panel,8_8_inch
max_framebuffers=2

# Don't have the firmware create an initial video= setting in cmdline.txt.
# Use the kernel's default instead.
# disable_fw_kms_setup=1
```

If you are running the Raspberry Pi OS with the default desktop, you can add the command to the autostart file so it launches when the X session starts:

1. Open (or create if it doesn’t exist) the autostart file:

   ```bash
   mkdir -p /home/robotronik/.config/autostart
   nano /home/robotronik/.config/autostart/kiosk.desktop
   ```

2. Add the following command:

   ```bash
   @/usr/bin/chromium-browser --kiosk http://localhost/robot --incognito --disable-extensions
   ```

✅ Correct setup for Debian 13 (GNOME / Wayland)

Create the file:

~/.config/autostart/kiosk.desktop

With this content:
```bash
[Desktop Entry]
Type=Application
Name=Kiosk Mode
Exec=bash -c "sleep 5 && /usr/bin/chromium --kiosk http://localhost/robot --incognito --no-first-run --no-default-browser-check --password-store=basic"
X-GNOME-Autostart-enabled=true
```
Then:
```bash
chmod +x ~/.config/autostart/kiosk.desktop
```
That’s the correct GNOME-compatible autostart format.


3. Save the file and reboot the system.

## ⚙️ Actions and Actuators

In the code, these elements are referred to as *banner*, *stocks*, *columns*, *platforms*, and *tribunes*. Defined in `constante.h`:

- **Stepper 1**: Platforms elevator
- **Stepper 2**: Multi-level elevator
- **Stepper 3**: Lower revolver
- **Servo 1**: Tribunes pusher
- **Servo 2**: Left platforms lifter
- **Servo 3**: Right platforms lifter
- **Servo 4**: Clamps
- **Servo 5**: String Claws
- **Servo 6**: Banner Front
- **Servo 7**: Banner Back
- **DC Motor 1**: Tribunes elevator

## 🌈 RGB Light Signals

- **SOLID**:
  - 🟢 *Green*: The robot has finished the match.
- **BLINKING**:
  - 🔴 *Red*: The program has failed. Restart the robot.
  - 🔵 *Blue*: The robot is ready to start as blue.
  - 🟡 *Yellow*: The robot is ready to start as yellow.
  - 🟣 *Purple*: The robot is in manual control mode.
- **RAINBOW**:
  - 🌈 The robot is waiting for user input.

## ✅ Match Checklist

- Position the robot.
- Setup mechanical parts.
- Choose color.
- Select strategy (verify on the live table).
- Ready to start.
