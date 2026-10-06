# 🤖 CDFR-Programme-Robot

Welcome to the **CDFR-Programme-Robot** project!

This project aims to develop a program to control a robot for the CDFR  event.

## 📖 Description

This program enables the robot to perform various tasks such as navigation, data collection, and more using a modular and extensible design.

## 🚀 Features

- **Navigation**: The robot can move through its environment using dedicated algorithms.
- **Data Collection**: The robot gathers and stores data from onboard sensors.
- **Communication**: The program supports communication with other systems or devices.
- **Vision**: Native C++ ArUco marker detection using OpenCV, running in-process (no external Python/REST service). Camera capture uses libcamera on the Raspberry Pi 5 (OV9281 behind the PiSP ISP) and OpenCV/V4L2 elsewhere.

## 🔧 Prerequisites

Before running the program, make sure you have installed the following dependencies:

```bash
sudo apt-get install cmake make gcc g++ ninja-build libopencv-dev libsqlite3-dev
```

To speed up compilation times massively, you can install CCache and MOLD:

```bash
sudo apt-get install ccache mold
```

For ARM (Raspberry Pi) compilation, install:

```bash
sudo apt-get install g++-aarch64-linux-gnu sqlite3
```

Cross-compiling for ARM also needs the `arm64` OpenCV, SQLite **and libcamera**
libraries. They cannot be installed with `apt` next to the `amd64` ones:
`libopencv-dev` is not `Multi-Arch: same`, so dpkg refuses to install
`libopencv-dev:arm64` alongside the version required by the local build. They
are instead downloaded and extracted into a local sysroot:

```bash
./scripts/fetch_arm64_sysroot.sh          # -> ~/aarch64-sysroot
```

`build.sh build_arm` uses it automatically. Set `ARM64_SYSROOT` to point at a
sysroot extracted somewhere else.

For debugging, install:

```bash
sudo apt install gdbserver
```

## 📥 Installation

1. **Do not clone this repository by itself!**  
   Instead, clone the main CDFR repository with the `--recursive` flag to include all submodules:

   ```bash
   git clone git@github.com:robotronik/CDFR.git --recursive
   ```

2. Navigate to the CDFR-Programme-Robot directory:

   ```bash
   cd informatique/CDFR-Programme-Robot/
   ```

3. Switch to the `main` branch and update the project:

   ```bash
   git checkout main
   git pull
   ```

4. (Optional) You may want to setup the LSP Server (clangd) if you are not on VSCode. To do so, run:
   
   ```bash
   bash build.sh setup-lsp
   ```

## 💻 Compilation

To compile the program on your machine, simply run:

```bash
bash build.sh build
```

To compile for ARM (Raspberry Pi) :

```bash
bash build.sh build_arm
```

To run tests:

```bash
bash build.sh tests
```

To clean the build files:

```bash
bash build.sh clean
```

## 🛠️ Compilation for Raspberry Pi

Ensure you have the necessary dependencies for ARM compilation.

To compile and deploy the program on your Raspberry Pi, first set up SSH key authentication. To copy your SSH key to the Raspberry Pi (replace `pi@192.168.1.47` with your Raspberry Pi’s address):

```bash
ssh-copy-id pi@192.168.1.47
```

Then compile and deploy with:

```bash
bash build.sh deploy
```

To clean up, run:

```bash
bash build.sh clean
```

On a new Raspberry Pi, configure I2C and serial communication via:

```bash
sudo raspi-config
```

## 📷 Camera Setup (Raspi with OV9281)

On Raspberry Pi 5 the OV9281 is behind the PiSP ISP, so the camera is captured
through **libcamera** rather than the plain V4L2 node (which only exposes raw
Bayer frames OpenCV cannot decode). The ARM build links libcamera automatically
when cross-compiling; the local build keeps using V4L2 so the tests run without
libcamera installed.

On the Pi itself (if building natively), install the runtime and headers:

```bash
sudo apt install libcamera-dev   # pulls libcamera0.2 on Ubuntu Noble, libcamera0.3 on Pi OS
```

The capture code also needs `libcamera/base/event_dispatcher.h` and
`libcamera/base/thread.h`, which Debian/Ubuntu and Raspberry Pi OS omit from
`libcamera-dev` despite them being part of the upstream API. When they are
missing the ARM build falls back to the minimal declarations under
`include/compat/`, so no extra package is required.

On Raspberry Pi 5, automatic camera detection must be disabled for the OV9281.
Run:

```bash
sudo nano /boot/firmware/config.txt
```

Add (or edit) the following lines near the top:

```bash
camera_auto_detect=0
dtoverlay=ov9281,cam0
```

If you plugged into the other CSI connector (CAM1), use `,cam1` instead.

Then save and exit (Ctrl+O, Enter, Ctrl+X) and reboot:

```bash
sudo reboot
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

## 🛰️ Mat de vision

The robot drives the vision mat over **HTTP (TCP)** through its REST API
(`src/mat/mat.cpp`, using cpp-httplib). The mat is expected at `mat.local:5000`
(override at compile time with `-DMAT_HOST=… -DMAT_PORT=…`).

| Call | Mat route | Purpose |
|---|---|---|
| `StartMat()` | `GET /start` | start detection (retries for 5 s) |
| `StopMat()` | `GET /stop` | stop detection |
| `getMapStatus()` | `GET /fleet/live` | opponent position + game elements; applied by `TableState::updateMapStatus()` |

The last payload received is available via `getMatTableData()`. The raw parsing
is covered by `tests/MatTest.cpp`.

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
