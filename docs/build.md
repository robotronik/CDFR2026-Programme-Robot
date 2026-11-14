# Build

[`build.sh`](../build.sh) runs every step inside Docker:

- The workspace is mounted at its own path, so build outputs appear directly on
  the host and file ownership is preserved.
- The images are defined in [`docker/Dockerfile.x86_64`](../docker/Dockerfile.x86_64)
  and [`docker/Dockerfile.arm64`](../docker/Dockerfile.arm64). They contain
  OpenCV, SQLite, libcamera (ARM) and the `drive_interface.h` / `protocol.h`
  headers. An image is rebuilt automatically when its Dockerfile changes;
  `./build.sh images` forces a rebuild.
- CI caches the image layers in GHCR: `build.sh` pulls `cdfr-builder-<arch>:cache`
  and builds with it as `--cache-from`. Only `main` pushes the cache back, so
  branches read it but can't overwrite it. Set `DOCKER_CACHE_REGISTRY` to use it locally.

## Outputs

The executable is `build/<arch>/programCDFR`. Each build directory also holds the
runtime bundle shipped with it: `html/`, `data/` (camera calibration files) and
`autoRunInstaller.sh`. CI zips these as the `programCDFR-<arch>` artifacts.

Local x86_64 builds have no hardware: they use the emulated I2C, disable the
lidar and run the API in test mode.

`./build.sh run` runs the program as root inside the container, so that
`--cap-add=SYS_NICE` can raise its real-time priority. The files it writes (the
`log/` directory) are therefore owned by root on the host; `./build.sh clean`
deletes them through the container.

## Windows

`build.bat` (a launcher for [`build.ps1`](../build.ps1)) drives the same images
as `build.sh`. Differences:

- The repository is mounted at `/work` inside the container; outputs still
  appear under `build\<arch>`.
- `build.bat deploy` uses the SSH keys in `%USERPROFILE%\.ssh`, mounted read-only
  into the container. `ssh-copy-id` does not exist on Windows: append your public
  key to the robot's `~/.ssh/authorized_keys`. A passphrase-protected key needs
  an SSH agent (`ssh-agent` + `ssh-add`).
- `compile_commands.json` is not exported, because its symlink cannot be written
  reliably on a Windows bind mount. For clangd, use the dev container.

## Dev container

VS Code / CLion can open the project in the x86_64 image through
[`.devcontainer/devcontainer.json`](../.devcontainer/devcontainer.json).
