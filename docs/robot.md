# Robot

## Actuators

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
