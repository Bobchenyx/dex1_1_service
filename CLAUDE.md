# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

`dex1_1_service` is a serial-to-DDS bridge for the Unitree Dex1-1 parallel two-finger gripper. Each gripper is driven by a single Unitree M4010 motor; up to two grippers (left + right) can be served from one process. Built for the Unitree G1's PC2 (NVIDIA Jetson Orin NX, aarch64); also builds on x86_64 Linux.

## Build

```bash
mkdir build && cd build
cmake ..              # Release by default; use -DCMAKE_BUILD_TYPE=Debug for debug
make -j6
```

Binaries are emitted to `bin/` (set via `CMAKE_RUNTIME_OUTPUT_DIRECTORY`), not `build/`. The README references `build/` paths but the actual artifacts land in `bin/`.

Build targets (defined in `CMakeLists.txt`):
- `dex1_1_gripper_server` — main service (`main.cpp`)
- `test_dex1_1_gripper_server` — sine-wave test client (`test/test_gripper.cpp`)
- `test_dex1_1_gripper_server2` — alternate test (`test/test_gripper2.cpp`)

### Python clients (PC-side, not built by CMake)

Under `test/` there are Python scripts meant to run on a PC (or other host) that's network-connected to the G1 where the server is already running. They depend only on `unitree_sdk2_python` (pip-installable) + stdlib — **no C++ build, no `unitree_sdk2` C++ install needed**, no serial port access, no `sudo`. Run them with `-n <iface>` pointing at the network on the `192.168.123.*` subnet.

- `test/test_gripper_sub.py` — state subscriber, prints FPS. Sanity-check that the DDS link is up.
- `test/test_gripper_pub.py` — sine-wave demo. Mirrors `test/test_gripper.cpp` behavior.
- `test/test_gripper_goto.py` — goto-target demo with per-side targets (`-l <q> -r <q>`).

Usage details and expected output live in `test/README.md`. All three use the same DDS-client pattern modeled after the canonical `Dex1_1_Gripper_Controller` in the `xr_teleoperate` repo (`~/Workspace/dev/xr_teleoperate/teleop/robot_control/robot_hand_unitree.py:232-390`).

Shared conventions across pub/goto scripts:
- `kp=5.0, kd=0.05`, 200 Hz control loop, no `mode` field (server's `MotorUnit::loop_` ignores it and forces FOC).
- `DELTA_GRIPPER_CMD = 0.18 rad/cycle` per-step velocity clamp — the load-bearing safety mechanism. Caps slew at ~36 rad/s and matches the reference controller verbatim.
- Closed-loop: state is re-read every control cycle, and the next command is clamped to `q_current ± DELTA_GRIPPER_CMD`. Smooth start from any initial position.
- Ctrl+C exits without sending zero/brake commands. Server detects subscription timeout (~1 s) and enters BRAKE mode (`main.cpp:64-68`). **Do not** add an active-stop step on exit — let the server contract do it.
- Gripper q range is `[0.0, 5.4]` rad (~0 closed, ~5.4 open). Sine demo's peak (5.5) intentionally exceeds the nominal max by 0.1 rad; mechanical/PD limits absorb it.

### Build dependencies

System packages: `libserialport-dev`, `libspdlog-dev`, `libboost-all-dev`, `libyaml-cpp-dev`, `libfmt-dev`.

External SDK that must be installed system-wide before building: [`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2) (provides `unitree/idl/go2/MotorCmds_.hpp`, `MotorStates_.hpp`, DDS headers under `/usr/local/include/ddscxx`, and links against `unitree_sdk2 ddsc ddscxx`). Missing this SDK is the #1 build failure — see README FAQ.

Vendored shared libs in `lib/`: `libUnitreeMotorSDK_Arm64.so` / `libUnitreeMotorSDK_Linux64.so` — CMake selects automatically based on `CMAKE_HOST_SYSTEM_PROCESSOR`. Also vendored: offline `.deb` files for `libserialport` (arm64) for situations where apt cannot reach the package.

## Run

```bash
sudo ./bin/dex1_1_gripper_server [-n eth0] [-c]
sudo ./bin/test_dex1_1_gripper_server [-n eth0] [-l] [-r]
```

`sudo` is required (raw access to `/dev/ttyUSB*` and DDS network init). The `-n` / `--network` argument is the DDS network interface (default `eth0`). When a USB hub adds extra Ethernet, `eth0` may no longer be the `192.168.123.*` interface — check with `ip addr` and override `-n` accordingly. `setup_autostart.sh` hard-codes no `-n` flag; edit that file's `ExecStart` line if the default interface is wrong.

Calibration: `sudo ./bin/dex1_1_gripper_server -c`. Operator must physically close each gripper, then press `s` + Enter per motor. Calibration writes to motor firmware; only re-run when needed.

Auto-start: `bash setup_autostart.sh` installs a systemd unit `dex1_gripper.service` (User=unitree, Group=dialout). Manage with `systemctl {status,restart,stop,disable} dex1_gripper.service` and `journalctl -u dex1_gripper.service -f`.

## Architecture

Single-process service. Pipeline (per motor):

```
DDS topic (cmd) → SubscriptionBase → MotorUnit::loop_() @ 500 Hz
                                          ↓
                                     SerialPort::sendRecv (blocks on UART)
                                          ↓
                                     RealTimePublisher → DDS topic (state)
```

Key files:

- `main.cpp` — three concerns colocated:
  1. `getAvailableSerialPorts()` enumerates `/dev/ttyUSB*` and `/dev/ttyCH343USB*` (newer serial hub).
  2. `Dex1GripperServer::detectMotors_()` probes IDs **0 and 1** on every discovered port by issuing a FOC-mode `sendRecv` and recording which (port, id) pairs respond. **Motor ID hard-codes side**: `id == 0` → right (`rt/dex1/right/{cmd,state}`), `id == 1` → left (`rt/dex1/left/{cmd,state}`). Retries up to 3× with 50 ms backoff before exiting.
  3. `MotorUnit` owns one `SerialPort` + DDS pub/sub + `RecurrentThread` (2000 µs period). On subscription timeout it forces `BRAKE` mode with zeroed gains; otherwise it scales user-space q/dq/kp/kd/tau by the motor's gear ratio before writing to the wire, and inverse-scales the read-back state before publishing. **Gear-ratio scaling lives in `MotorUnit::loop_()`** — users publish/subscribe in joint space, not motor space.

- `include/param.h` — Boost.program_options wrappers `param::helper` (server) and `param::helper_test` (test client). Also defines `param::VERSION`. Both helpers set spdlog level based on `NDEBUG`.

- `include/dds/{Publisher.h,Subscription.h}` — local wrappers around `unitree::robot` channel pub/sub providing `RealTimePublisher` (trylock + unlockAndPublish) and `SubscriptionBase` (isTimeout, wait_for_connection). The server uses these for `MotorCmds_` / `MotorStates_` IDL messages from `unitree_sdk2`.

- `include/{serialPort,unitreeMotor,IOPort,crc}/` — vendored motor SDK headers (M4010 framing, CRC, IO abstraction). Treat as third-party — do not modify unless syncing with upstream.

- `urdf/` — Dex1-1 URDF/meshes for visualization (not consumed by the server itself).

### DDS topic contract

| Side  | Cmd topic              | State topic              | Motor ID |
|-------|------------------------|--------------------------|----------|
| Right | `rt/dex1/right/cmd`    | `rt/dex1/right/state`    | 0        |
| Left  | `rt/dex1/left/cmd`     | `rt/dex1/left/state`     | 1        |

Cmd/state messages are `unitree_go::msg::dds_::MotorCmds_` / `MotorStates_` with a single-element `cmds()` / `states()` vector. Fields used: `mode`, `kp`, `kd`, `q`, `dq`, `tau` (cmd); `q`, `dq`, `tau_est` (state). Side identity is determined entirely by the motor ID stored in firmware — there is no per-port configuration file.

## Gotchas

- The "Silence begin/end" block around `detectMotors_` in `main.cpp` is intentionally commented out (see commit `4c30bc9`). It was a `stdout` redirect to `/dev/null` to suppress noise from `serial->sendRecv()` during probing; keep it commented unless debugging.
- The aarch64 SDK `.so` is loaded via `link_directories(${CMAKE_SOURCE_DIR}/lib)` at build time, and at runtime via `LD_LIBRARY_PATH` set by the systemd unit. If running the binary outside systemd, the `lib/` path is found because `link_directories` also embeds an rpath for the build tree — but copying `bin/dex1_1_gripper_server` elsewhere will break this.
- `setup_autostart.sh` runs the service as user `unitree` in group `dialout` — verify those exist on the target system.
