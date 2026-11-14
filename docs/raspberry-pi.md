# Raspberry Pi

The robot runs **Raspberry Pi OS Trixie (64-bit)**. The arm64 build image is
Debian 13, so the binary links against the libraries the Pi ships: GCC 14 /
glibc 2.41, OpenCV 4.10 and the Raspberry Pi libcamera fork (`libcamera.so.0.7`).
It is `aarch64` and cannot run on a 32-bit OS.

## Setup

The deployed bundle does not include shared libraries. Install them once:

```bash
sudo apt update
sudo apt install -y \
  libsqlite3-0 \
  libopencv-objdetect410 libopencv-imgcodecs410 libopencv-calib3d410 \
  libopencv-features2d410 libopencv-imgproc410 libopencv-core410 \
  libcamera0.7 libcamera-ipa libpisp1
```

`libopencv-objdetect410` provides ArUco; `libcamera0.7` comes from
`archive.raspberrypi.com`.

Then enable the I2C bus and the UART (used by the actuators and the lidar) in:

```bash
sudo raspi-config
```

## Deploy

The target is set in [`cmake/deploy.cmake`](../cmake/deploy.cmake):
`robotronik@192.168.8.108` by default, installed to
`/home/robotronik/CDFR/arm_bin` (cache variables `PI_USER`, `PI_HOST`, `PI_DIR`,
`PI_DEST`). Copy your SSH key to the robot, then deploy:

```bash
ssh-copy-id robotronik@192.168.8.108
./build.sh deploy
```

Deploy copies the bundle and (re)installs the `programCDFR` service with
`autoRunInstaller.sh`.

With the robot running and on the same network, the UI is at
<http://raspitronik.local>.

## Service logs and restart

```bash
journalctl -b -u programCDFR --output=cat     # logs since boot
journalctl -u programCDFR -f --output=cat     # follow the logs
systemctl list-units --type=service           # list active services
sudo systemctl daemon-reload
sudo systemctl restart programCDFR
```

To keep the logs across reboots, add to `/etc/systemd/journald.conf`:

```ini
[Journal]
Storage=persistent
SyncIntervalSec=2s
```

## Debugging with VS Code

1. Connect your PC to the same network as the robot.
2. Press F5: [`.vscode/tasks.json`](../.vscode/tasks.json) starts `gdbserver` on
   the robot and [`.vscode/launch.json`](../.vscode/launch.json) attaches to it.

Both use the hostname `raspitronik.local`; edit them only if the robot has
another hostname.

## Touchscreen kiosk

If the clock is wrong, SSL fails. Disable NTP and set the current date first:

```bash
sudo timedatectl set-ntp false
sudo timedatectl set-time 'YYYY-MM-DD HH:MM:SS'
sudo apt-get update
sudo apt-get upgrade -y
```

Install the packages and start Chromium in kiosk mode:

```bash
sudo apt install libcamera-apps xorg openbox chromium
export DISPLAY=:0
sudo startx /usr/bin/chromium --noerrdialogs --kiosk http://localhost/robot --incognito --disable-extensions --no-sandbox
```

For the portrait display, add to `/boot/firmware/config.txt`:

```ini
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

To start the kiosk at session start, create `~/.config/autostart/kiosk.desktop`:

```ini
[Desktop Entry]
Type=Application
Name=Kiosk Mode
Exec=bash -c "sleep 5 && /usr/bin/chromium --kiosk http://localhost/robot --incognito --no-first-run --no-default-browser-check --password-store=basic"
X-GNOME-Autostart-enabled=true
```

```bash
chmod +x ~/.config/autostart/kiosk.desktop
```

Then reboot.
