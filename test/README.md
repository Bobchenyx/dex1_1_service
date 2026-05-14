# `test/` — gripper test clients

C++ test programs (`test_gripper.cpp`, `test_gripper2.cpp`) are built by the top-level `CMakeLists.txt` and end up in `bin/` — see the repo README §2 for those.

This file documents the **Python scripts**, which run on any host networked to the G1 where the C++ server is already running (e.g. a lab PC plugged into the robot via Ethernet on the `192.168.123.*` subnet).

## Requirements

- Python 3, `pip install unitree_sdk2_python`
- PC on the same subnet as the G1, and the G1 running `dex1_1_gripper_server` (auto-start via `setup_autostart.sh` or manually)
- Find the right network interface name: `ip -br addr | grep 192.168.123` — pass it via `-n <iface>` (e.g. `-n enp0s31f6`). Default is `eth0`, which is usually wrong on lab PCs.
- **No `sudo` needed**, no C++ toolchain, no `unitree_sdk2` C++ install. Pure DDS pub/sub over the network.

## Scripts

### `test_gripper_sub.py` — state subscriber (read-only)

Subscribes to `rt/dex1/{left,right}/state` and prints the receive FPS. Use this first to verify the DDS link.

```bash
python test/test_gripper_sub.py -n enp0s31f6
```

Expected: ~100 ms of `fps: 0.000` while DDS discovery handshakes, then both sides stabilize at **~500 fps** (server's `MotorUnit::loop_` runs at 2000 µs / 500 Hz, see `main.cpp:59`). Ctrl+C to stop. Does **not** move the gripper.

### `test_gripper_pub.py` — sine-wave demo

Drives one or both grippers along a sine trajectory (`q_center=3.0, amplitude=2.5, period=3s`). Mirrors the behavior of the C++ `test_dex1_1_gripper_server`.

```bash
# right only
python test/test_gripper_pub.py -n enp0s31f6 -r

# both
python test/test_gripper_pub.py -n enp0s31f6 -l -r
```

Expected:
- `initial q: {...}` printed on startup (current motor positions).
- Live `R= 3.456 L= 3.412` line, refreshed at ~200 Hz.
- Gripper(s) open and close visibly at a 3-second period.
- Sine phase is aligned to the initial q so the first cycle starts at the current position — **no startup lurch**.
- Ctrl+C: prints `stopping — server will brake on subscription timeout`. Gripper holds for ~1 s, then the server-side `isTimeout()` switches the motor to BRAKE and clears gains (`main.cpp:64-68`).

Constants live at the top of the file. The trajectory amplitude (range `[0.5, 5.5]`) intentionally clips slightly past the nominal mechanical max (5.4); kp=5 + the per-cycle 0.18 rad clamp keep this safe.

### `test_gripper_goto.py` — goto-target demo

Moves selected grippers to a target q and **holds** until Ctrl+C. Each side gets its own target — pick whichever sides you want to move.

```bash
# right hand only, open to 5.0
python test/test_gripper_goto.py -n enp0s31f6 -r 5.0

# both hands, asymmetric targets
python test/test_gripper_goto.py -n enp0s31f6 -l 1.0 -r 4.0

# both hands, close to 0.5
python test/test_gripper_goto.py -n enp0s31f6 -l 0.5 -r 0.5
```

Expected:
- `initial q: {...}, targets: {...}` on startup.
- Live `R= ... L= ...` line; each side slews independently toward its own target at the per-cycle clamp rate (0.18 rad/cycle ≈ 36 rad/s max).
- Total time to target ≈ `|target - initial_q| / 36` seconds. Sides do **not** synchronize arrival — far-from-target sides take longer.
- Once at target, command keeps publishing `q = target` every cycle so the server's PD loop holds position. **Script does not auto-exit** — releasing it (Ctrl+C) lets the server timeout into BRAKE, which drops holding torque.
- Out-of-range target (`<0.0` or `>5.4`) → immediate `SystemExit` with the offending side and value, before any DDS work.
- Neither `-l` nor `-r` given → `SystemExit("specify --left <q> and/or --right <q>")`.

## Shared design notes (apply to pub & goto)

These are documented at the top of `CLAUDE.md` too; restated here for users who land in `test/` first:

- **Per-cycle velocity clamp** `DELTA_GRIPPER_CMD = 0.18 rad` is the load-bearing safety: even if the target is far from current q, the command can only move 0.18 rad per cycle (200 Hz → 36 rad/s max). Borrowed verbatim from `~/Workspace/dev/xr_teleoperate/teleop/robot_control/robot_hand_unitree.py:321-328`.
- **No `mode` field** in `MotorCmds_` — the server (`main.cpp:71`) ignores `cmd.mode` and forces FOC when subscription is alive.
- **Ctrl+C strategy**: stop publishing, let the server's 1 s subscription-timeout fire and brake the motor. Do not send zero/brake commands explicitly — the server handles it cleanly.
- **No motor ID configuration**: side identity is baked into the motor's firmware ID (0 = right, 1 = left). Topics are fixed at `rt/dex1/{left,right}/{cmd,state}`.

## Reference

Canonical Python controller for the same gripper, including the velocity-clamp pattern these scripts borrow from:
`~/Workspace/dev/xr_teleoperate/teleop/robot_control/robot_hand_unitree.py:232-390` (`Dex1_1_Gripper_Controller`).
