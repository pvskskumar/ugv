#!/usr/bin/env python3
"""
UGV 4-Wheel Manual Throttle + Feedback Compensation (v3)
for ODrive S1 (BotWheel 24 V, 8.7 KV motors)

Features:
 - Keyboard throttle (A/Z fine, Shift+A/Z coarse)
 - Live per-wheel velocity & distance feedback
 - Encoder scaling correction, slip compensation, noise filtering
 - Safe stop / idle / graceful exit
"""

import can, struct, time, sys, termios, tty, select, math

# ========= USER CONFIGURATION =========
NODE_IDS = [1, 2, 3, 4]
VEL_SIGN = {1:-1, 2:+1, 3:-1, 4:+1}     # right-side inverted
GAIN = {1:1.00, 2:1.00, 3:0.98, 4:1.03} # empirical slip/diameter trim

WHEEL_DIAMETER_M = 0.171
CIRCUM = math.pi * WHEEL_DIAMETER_M

CAN_CHANNEL = "can0"
PRINT_PERIOD = 0.5

VEL_MAX_RAD_S = 22.0          # ≈ 210 RPM @ 24 V
VEL_FINE_STEP = 0.5
VEL_COARSE_STEP = 2.0
RAMP_SMOOTH = 0.25

MAX_JUMP_M = 0.03             # ignore >3 cm encoder jump
SYNC_BLEND = 0.02             # small re-alignment blend factor

AXIS_STATE_CLOSED_LOOP = 8
AXIS_STATE_IDLE = 1
CMD_SET_AXIS_STATE  = 0x07
CMD_SET_INPUT_VEL   = 0x0D
CMD_GET_ENCODER_EST = 0x09
# =====================================


# ---------- CAN helper functions ----------
def open_bus():
    return can.interface.Bus(interface="socketcan", channel=CAN_CHANNEL)

def mid(n, c): return (n << 5) | c

def send(bus, n, c, d=b""):
    bus.send(can.Message(arbitration_id=mid(n, c), data=d, is_extended_id=False))

def set_axis_state(bus, ids, state):
    for nid in ids:
        send(bus, nid, CMD_SET_AXIS_STATE, struct.pack("<I", state))

def poll_encoders(bus, ids, timeout=0.1):
    """Send parallel encoder requests & collect replies."""
    for nid in ids:
        send(bus, nid, CMD_GET_ENCODER_EST, b"")
    results = {}
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = bus.recv(timeout=timeout)
        if not msg:
            break
        for nid in ids:
            if msg.arbitration_id == mid(nid, CMD_GET_ENCODER_EST):
                pos, vel = struct.unpack("<ff", msg.data[:8])
                results[nid] = (pos, vel)
        if len(results) == len(ids):
            break
    return results

def get_key():
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


# ---------- MAIN ----------
def main():
    bus = open_bus()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        # Arm all axes
        set_axis_state(bus, NODE_IDS, AXIS_STATE_CLOSED_LOOP)
        time.sleep(0.5)
        print("\n🦾 UGV 4-Wheel — Manual Throttle + Feedback Compensation")
        print("   a/z fine ±0.5   A/Z coarse ±2.0   s stop   q quit\n")

        # --- Initialize feedback state ---
        enc_init = poll_encoders(bus, NODE_IDS)
        pos_prev = {n: enc_init.get(n, (0, 0))[0] for n in NODE_IDS}
        dist = {n: 0.0 for n in NODE_IDS}

        target_vel = 1.0
        current_vel = 0.0
        t0 = time.time()
        last_print = t0

        print("✅ All ODrives in CLOSED_LOOP_CONTROL\n")

        # --- Control loop ---
        try:
            while True:
                k = get_key()
                if k:
                    if k == "a":
                        target_vel = min(VEL_MAX_RAD_S, target_vel + VEL_FINE_STEP)
                    elif k == "z":
                        target_vel = max(0.0, target_vel - VEL_FINE_STEP)
                    elif k == "A":
                        target_vel = min(VEL_MAX_RAD_S, target_vel + VEL_COARSE_STEP)
                    elif k == "Z":
                        target_vel = max(0.0, target_vel - VEL_COARSE_STEP)
                    elif k.lower() == "s":
                        target_vel = 0.0
                        print("🟥 STOP")
                    elif k.lower() == "q":
                        print("👋 QUIT")
                        break
                    print(f"⚙️ Target {target_vel:.2f} rad/s")

                # Smooth ramp
                current_vel += (target_vel - current_vel) * RAMP_SMOOTH

                # Command all motors
                for nid in NODE_IDS:
                    send(bus, nid, CMD_SET_INPUT_VEL,
                         struct.pack("<ff", current_vel * VEL_SIGN[nid], 0.0))

                # Encoder feedback
                encs = poll_encoders(bus, NODE_IDS, timeout=0.05)
                for nid, (p, v) in encs.items():
                    delta_turns = (p - pos_prev[nid]) * VEL_SIGN[nid]
                    delta_m = delta_turns * CIRCUM * GAIN[nid]
                    # Filter unrealistic jumps
                    expected = abs(current_vel * PRINT_PERIOD * WHEEL_DIAMETER_M / 2)
                    if abs(delta_m) > max(expected * 3, MAX_JUMP_M):
                        delta_m = 0.0
                    dist[nid] += delta_m
                    pos_prev[nid] = p

                # Normalize distances slightly
                avg_d = sum(dist.values()) / len(dist)
                for nid in NODE_IDS:
                    dist[nid] += (avg_d - dist[nid]) * SYNC_BLEND

                # Print telemetry
                if time.time() - last_print > PRINT_PERIOD:
                    msg = " | ".join(
                        [f"N{n}:{dist[n]*100:6.2f} cm ({encs.get(n,(0,0))[1]*2*math.pi:5.2f} rad/s)"
                         for n in NODE_IDS])
                    print(f"[{time.time()-t0:5.1f}s] cmd={current_vel:5.2f} rad/s → avg={avg_d*100:6.2f} cm  {msg}")
                    last_print = time.time()

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n🟥 Manual break received")

        # --- Stop safely ---
        for nid in NODE_IDS:
            send(bus, nid, CMD_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))
        time.sleep(0.2)
        set_axis_state(bus, NODE_IDS, AXIS_STATE_IDLE)
        print("\n🟢 Motors idled safely.")

        # --- Final summary ---
        print("\n========= RESULT SUMMARY =========")
        for n in NODE_IDS:
            print(f"Node {n}: {dist[n]*100:7.2f} cm")
        print("=================================")

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        set_axis_state(bus, NODE_IDS, AXIS_STATE_IDLE)
        bus.shutdown()
        print("🟡 Bus closed safely\n")


# ---------- Entry ----------
if __name__ == "__main__":
    main()

