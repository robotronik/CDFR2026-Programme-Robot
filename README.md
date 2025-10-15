# 🤖 CDFR-Programme-Robot

Welcome to the **CDFR-Programme-Robot** project!

This project aims to develop a program to control a robot for the CDFR  event.

## 📖 Description

This program enables the robot to perform various tasks such as navigation, data collection, and more using a modular and extensible design.

## 🚀 Features

- **Navigation**: The robot can move through its environment using dedicated algorithms.
- **Data Collection**: The robot gathers and stores data from onboard sensors.
- **Communication**: The program supports communication with other systems or devices.

## 🔧 Prerequisites

Before running the program, make sure you have installed the following dependencies:

```bash
sudo apt-get install make gcc g++ python3-venv libasio-dev
```

For ARM (Raspberry Pi) compilation, install:

```bash
sudo apt-get install g++-aarch64-linux-gnu
```

*Note:* You do not need to install `libi2c-dev_4.3-2`.

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

## 💻 Compilation

To compile the program on your machine, simply run:

```bash
make
```

To run tests:

```bash
make tests
```

To clean the build files:

```bash
make clean
```

## 🛠️ Compilation for Raspberry Pi

Ensure you have the necessary dependencies for ARM compilation.

To compile and deploy the program on your Raspberry Pi, first set up SSH key authentication. To copy your SSH key to the Raspberry Pi (replace `pi@192.168.1.47` with your Raspberry Pi’s address):

```bash
ssh-copy-id pi@192.168.1.47
```

Then compile and deploy with:

```bash
make deploy
```

To clean up, run:

```bash
make clean-all
```

On a new Raspberry Pi, configure I2C and serial communication via:

```bash
sudo raspi-config
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

To launch Chromium in kiosk mode on the robot, execute:

```bash
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
dtoverlay=vc4-kms-dsi-waveshare-panel,8_8_inch
```

If you are running the Raspberry Pi OS with the default desktop, you can add the command to the autostart file so it launches when the X session starts:

1. Open (or create if it doesn’t exist) the autostart file:

   ```bash
   /home/pi/.config/autostart/kiosk.desktop
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
Exec=bash -c "sleep 5 && /usr/bin/chromium --kiosk http://localhost/robot --incognito --disable-extensions"
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
- Elevator at the bottom position.
- Clamp at the bottom position.
- Choose color.
- Select strategy (verify on the live table).
- Ready to start.