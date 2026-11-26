# Robot

## Actuators

All actuators are driven by the 2027 Arduino board: servos through the PCA9685
PWM outputs, pumps and electrovalves as fully on/off outputs.

- **Servo 1**: Turn Left Tower
- **Servo 2**: Graal Door Lift Left
- **Servo 3**: Turn Right Tower
- **Servo 4**: Graal Door Lift Right
- **Servo 5**: Back Rail
- **Servo 6**: Fire Bullets
- **Pump 1–6**: Not decided yet
- **EV 1–10**: Not decided yet

## LED signals

- **SOLID**:
  - 🟢 *Green*: The robot has finished the match.
- **BLINKING**:
  - 🔴 *Red*: The program has failed. Restart the robot.
  - 🔵 *Blue*: The robot is ready to start as blue.
  - 🟡 *Yellow*: The robot is ready to start as yellow.
  - 🟣 *Purple*: The robot is in manual control mode.
- **RAINBOW**:
  - 🌈 The robot is waiting for user input.

## Debug sounds

The program plays short sounds to mark startup, match end and shutdown, and any
`debugSound::play("name.wav")` you add for debugging a code path (samples in
[`data/sounds/`](../data/sounds)). They are heard on a USB speaker plugged into the
robot (the Pi 5 has no audio jack) and on a laptop running the Docker image:
`./build.sh run` plays the startup sound on launch. See
[AUDIO.md](../AUDIO.md).

## Match checklist

- Position the robot.
- Setup mechanical parts.
- Choose color.
- Select strategy (verify on the live table).
- Ready to start.
