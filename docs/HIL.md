# HIL — hardware checks on the bench

The unit tests in `tests/` replay recorded data, so they never touch a device.
The HIL checks close that gap: they start the real program on the robot (or on a
bench wired like it) and prove that the camera, the lidar and the two boards
answer. Nothing on the robot is commanded to move.

## Running them

The checks live in the `TEST` state and are enabled by a boot argument, so the
binary is the same one the robot runs:

```bash
sudo ./programCDFR --unit-test
```

Without the argument the program behaves as usual: `TEST` is still the host
API-test state and runs no checks.

The program performs the checks once, logs the result of each one and then exits
through the normal shutdown path. The results are only in the log — the exit
status is not used — so look for the `HIL` lines:

```
===== HIL hardware checks =====
HIL PASS  camera: a frame arrives
HIL PASS  camera: exposure value round-trip
...
HIL SKIP  lidar: not built on this target (DISABLE_LIDAR)
===== HIL hardware checks done: 0 failure(s) =====
```

Each check is `PASS`, `FAIL`, or `SKIP` when the subsystem is not part of the
build (the host build emulates I2C and has no lidar).

## What is checked

| Subsystem | Checks |
|---|---|
| Camera | A frame arrives; the exposure, contrast and brightness values write and read back; frames still arrive after those writes (the previous values are restored). On the Pi this exercises libcamera, the format negotiation and the grayscale conversion. |
| Lidar | The motor spins and a scan arrives; the point count is within the buffer; enough ranges are inside the sensor's measurable window; ranges are not all identical (which is what a frozen motor looks like). |
| I2C — Arduino | A sensor read, a stepper position read and a servo position read all answer. Read-only. |
| I2C — drive board | `update()` reads the motion and the target back; the position is finite. Read-only, and the protocol version is also checked when the board is opened at startup. |

## What is *not* checked

The bench is assumed to move nothing, so anything that waits for feedback to
change is out of scope:

- Motor, servo and stepper travel, and therefore the `RUN` state, navigation and
  every action that waits for a position to be reached.
- Anything that needs the table, the opponent, or the MAT referee. The MAT is not
  contacted at all in this mode.
- Numerical accuracy. A camera with little in view cannot reproduce the golden
  values in `tests/data`; that stays the job of the replay tests.

The camera settings check proves the calls reach the capture layer and that the
pipeline keeps running. It does not prove the sensor changed its exposure — with
a lens cap the image may not move measurably.

## Requirements

- A Pi running the same OS as the build image (Raspberry Pi OS Trixie, glibc
  >= 2.41), with the runtime libraries installed — the bundle ships no `.so`.
- Root, for port 80 and `CAP_SYS_NICE`.
- The lidar on `/dev/ttyAMA0` and both boards on `/dev/i2c-1`. If a board cannot
  be opened the program aborts during startup, before the checks run.
- Stop the installed service first: it holds port 80, the camera and the lidar.

## CI

The `bench` job in `.github/workflows/c-cpp.yml` runs the hardware checks:

```
build-and-test (GitHub-hosted)  ->  artifact programCDFR-arm64  ->  bench job (self-hosted, label "bench")
```

It waits for the build, downloads the arm64 bundle that run just produced, unpacks
it, stops the installed service, runs `./programCDFR --unit-test`, greps the log,
and always restarts the service. It runs on **every branch**, for pushes and for
pull requests opened from this repository; a fork PR is skipped so its artifact
never executes on the bench. It lives in the same workflow as the build, so its
result appears next to it in the Actions run.

It is deliberately **not** a `workflow_run` workflow: that event only fires when
the workflow file exists on the default branch, which would have kept the bench
dormant on feature branches. A `push`/`pull_request` job uses the YAML from the
commit being built, so it is active as soon as the branch is pushed.

If the bench is switched off, the job waits in the queue instead of failing —
cancel the run if it is in the way.

### Setting up the bench runner

Check the Pi before anything else:

```bash
uname -m                  # must print aarch64
ldd --version | head -1   # must be >= 2.41 (Raspberry Pi OS Trixie)
```

Register the agent as `robotronik` (the user `./build.sh deploy` already uses):

```bash
# Registration token: repo -> Settings -> Actions -> Runners -> New self-hosted runner
# (valid about an hour), or via the CLI:
gh api -X POST repos/robotronik/CDFR2026-Programme-Robot/actions/runners/registration-token --jq .token

# Install the runner (2.338.0 is the release this was written against)
sudo mkdir -p /opt/actions-runner && sudo chown robotronik:robotronik /opt/actions-runner
cd /opt/actions-runner
curl -fsSLO https://github.com/actions/runner/releases/download/v2.338.0/actions-runner-linux-arm64-2.338.0.tar.gz
echo "628b4a7258487b80c1d3c221095a7ded349f5ae0175fd3dfab708d07428f041b  actions-runner-linux-arm64-2.338.0.tar.gz" | sha256sum -c -
tar xzf actions-runner-linux-arm64-2.338.0.tar.gz

# Register. `bench` is the label the bench job matches; self-hosted/Linux/ARM64 are added
# automatically.
./config.sh --url https://github.com/robotronik/CDFR2026-Programme-Robot \
            --token "$TOKEN" --name bench-pi --labels bench --unattended

# Run it as a service so it survives reboots
sudo ./svc.sh install robotronik
sudo ./svc.sh start && sudo ./svc.sh status
```

The agent only makes **outbound** connections to github.com on port 443 and polls
for work, so nothing has to be opened on the router — this is why it works behind
the bench's NAT or hotspot.

### Permissions

The job runs `sudo systemctl stop|start programCDFR` and
`sudo ./programCDFR --unit-test`, so the agent needs one of these:

**Simplest — run the agent as root.** No sudoers needed, and `sudo` in the
workflow becomes a no-op:

```bash
RUNNER_ALLOW_RUNASROOT=1 ./config.sh --url https://github.com/robotronik/CDFR2026-Programme-Robot \
    --token "$TOKEN" --name bench-pi --labels bench --unattended
sudo ./svc.sh install root && sudo ./svc.sh start
```

A root agent on a public repository is the highest-risk configuration, and the
`if:` gate in the bench job is what makes it acceptable. Keep that
gate.

**Least privilege — stay as `robotronik`:**

```bash
SYSTEMCTL=$(command -v systemctl)
sudo tee /etc/sudoers.d/hil-agent >/dev/null <<EOF
robotronik ALL=(root) NOPASSWD: $SYSTEMCTL stop programCDFR, $SYSTEMCTL start programCDFR
robotronik ALL=(root) NOPASSWD: /opt/actions-runner/_work/*/*/bench/programCDFR
EOF
sudo chmod 0440 /etc/sudoers.d/hil-agent
```

The unit name must match the one `autoRunInstaller.sh` creates, which is
`programCDFR.service`.

### Checking it works

- `sudo ./svc.sh status` reports the service as running.
- The repository's **Settings → Actions → Runners** lists `bench-pi` as *Idle*
  with the labels `self-hosted, Linux, ARM64, bench`.
- To watch a job live, stop the service and run `./run.sh` in the runner
  directory instead; the job output streams to the terminal.

The runner updates itself periodically, which needs write access to its own
directory — `/opt/actions-runner` is owned by `robotronik` for that reason.

