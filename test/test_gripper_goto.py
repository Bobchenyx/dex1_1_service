"""
Minimal Python example: move Dex1-1 gripper(s) to a target position.

Usage:
    python test/test_gripper_goto.py -n <iface> -l <q_left> -r <q_right>

Each given target must be in [0.0, 5.4] rad
(~0 fully closed, ~5.4 fully open; see robot_hand_unitree.py:321-328).

Ctrl+C to stop — server brakes the motor on subscription timeout (main.cpp:64-68).
"""
import argparse, time

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorCmds_, MotorStates_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__MotorCmd_

Q_MIN, Q_MAX = 0.0, 5.4
KP, KD, FPS = 5.0, 0.05, 200.0
DELTA_GRIPPER_CMD = 0.18

TOPICS = {
    "left":  ("rt/dex1/left/cmd",  "rt/dex1/left/state"),
    "right": ("rt/dex1/right/cmd", "rt/dex1/right/state"),
}


def make_cmd_msg():
    msg = MotorCmds_()
    msg.cmds = [unitree_go_msg_dds__MotorCmd_()]
    msg.cmds[0].kp = KP
    msg.cmds[0].kd = KD
    msg.cmds[0].dq = 0.0
    msg.cmds[0].tau = 0.0
    return msg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--network", default=None)
    ap.add_argument("-l", "--left",  type=float, default=None, metavar="Q",
                    help=f"left target q, range [{Q_MIN}, {Q_MAX}]")
    ap.add_argument("-r", "--right", type=float, default=None, metavar="Q",
                    help=f"right target q, range [{Q_MIN}, {Q_MAX}]")
    args = ap.parse_args()

    targets = {s: getattr(args, s) for s in ("right", "left") if getattr(args, s) is not None}
    if not targets:
        raise SystemExit("specify --left <q> and/or --right <q>")
    for side, q in targets.items():
        if not (Q_MIN <= q <= Q_MAX):
            raise SystemExit(f"{side} target {q} out of range [{Q_MIN}, {Q_MAX}]")

    ChannelFactoryInitialize(0, args.network)

    active = list(targets.keys())
    pubs, subs, msgs = {}, {}, {}
    for side in active:
        cmd_topic, state_topic = TOPICS[side]
        pubs[side] = ChannelPublisher(cmd_topic, MotorCmds_);  pubs[side].Init()
        subs[side] = ChannelSubscriber(state_topic, MotorStates_); subs[side].Init()
        msgs[side] = make_cmd_msg()

    q_current = {}
    deadline = time.time() + 2.0
    while time.time() < deadline and len(q_current) < len(active):
        for side in active:
            if side in q_current:
                continue
            m = subs[side].Read()
            if m is not None and m.states:
                q_current[side] = m.states[0].q
        time.sleep(0.01)
    missing = set(active) - set(q_current)
    if missing:
        raise SystemExit(f"no state received from {missing}; check -n and server status")
    print(f"initial q: {q_current}, targets: {targets}")

    dt = 1.0 / FPS
    try:
        while True:
            for side in active:
                m = subs[side].Read()
                if m is not None and m.states:
                    q_current[side] = m.states[0].q
                q_cmd = max(q_current[side] - DELTA_GRIPPER_CMD,
                            min(q_current[side] + DELTA_GRIPPER_CMD, targets[side]))
                msgs[side].cmds[0].q = q_cmd
                pubs[side].Write(msgs[side])
            line = " ".join(f"{s[0].upper()}={q_current[s]:6.3f}" for s in active)
            print(f"\r{line}", end="", flush=True)
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nstopping — server will brake on subscription timeout")
        return


if __name__ == "__main__":
    main()
