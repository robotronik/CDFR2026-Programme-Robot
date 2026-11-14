# 🤖 CDFR-Programme-Robot

The program that controls the Robotronik robot for the CDFR (Coupe de France de Robotique).

## 🔧 Prerequisites

- **Docker**, daemon running.
  Everything is compiled inside Docker images, so no compiler or library is
  needed on your machine.
- SSH access to the robot, only to deploy.

## 📥 Installation

1. **Do not clone this repository by itself!** Clone the main CDFR repository
   with `--recursive` so the `rplidar_sdk` submodule is checked out:

   ```bash
   git clone git@github.com:robotronik/CDFR.git --recursive
   ```

   If you already cloned it without `--recursive`:

   ```bash
   git submodule update --init --recursive
   ```

2. Go to the project directory:

   ```bash
   cd informatique/CDFR2026-Programme-Robot/
   ```

## 💻 Build and run

```bash
./build.sh build          # Build both x86_64 and arm64
./build.sh build x86_64   # Build a single target (x86_64 or arm64)
./build.sh run            # Build x86_64 and run it in the Docker image
./build.sh test           # Build x86_64 and run the CTest suite
./build.sh deploy         # Build arm64 and deploy it to the robot
./build.sh shell [arm64]  # Interactive shell in the x86_64 (or arm64) image
./build.sh clangd         # Language server in the x86_64 image (used by editors)
./build.sh images         # (Re)build the Docker images
./build.sh clean          # Remove the build/ directory
```

`./build.sh run` serves the UI on <http://localhost:8080> (set `CDFR_RUN_PORT` to
use another port).

**Windows:** use `build.bat` with the same commands (except `clangd`), with
Docker Desktop running. See [docs/build.md](docs/build.md#windows).

## 🧠 Editor (clangd)

Install the [clangd extension](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd);
[`.vscode/settings.json`](.vscode/settings.json) already runs it through
`./build.sh clangd`. After adding a source file, build once and restart clangd.

## 📚 Documentation

- [Build](docs/build.md) — how the Docker build works, outputs, Windows, dev container
- [Raspberry Pi](docs/raspberry-pi.md) — setup, deploy, logs, debugging, touchscreen
- [Robot](docs/robot.md) — actuators, LED signals, match checklist
