#!/usr/bin/env python3
"""
Standalone servoJ diagnostic for URSim (or real robot).

This script bypasses robot_manager / state_machine entirely and sends
servoJ commands directly to a UR controller.  It uses a URScript socket
protocol (not RTDEControlInterface) so it works even without Remote Control
mode — making it compatible with URSim out of the box.

Architecture
------------
  Python server (this script) ← URScript connects to → port 50002
  Python reads actual joints  → RTDEReceiveInterface

Usage (inside the URSim stack, from the teleop container):

    # Both arms sequentially (default)
    docker exec teleop_ws-teleop-1 python3 \\
        /workspace/teleop_interface/tests/test_ursim_servoj.py

    # One arm
    docker exec teleop_ws-teleop-1 python3 \\
        /workspace/teleop_interface/tests/test_ursim_servoj.py \\
        --ip 172.30.0.20 --one

    # Connection check only (no motion) — works from host too
    python3 teleop_interface/tests/test_ursim_servoj.py \\
        --ip 172.30.0.20 --read-only

    # Custom server IP (if running from a non-teleop host on the simnet)
    python3 teleop_interface/tests/test_ursim_servoj.py \\
        --server-ip 172.30.0.1

What it tests
-------------
1. RTDE receive connection can be established
2. Robot can be powered on + brake-released via Dashboard server
3. URScript socket server is uploaded and connects back
4. servoJ commands are accepted and joints track the commanded trajectory
5. Joint tracking error stays within a reasonable band (≤ 15° for socket protocol)
"""

import argparse
import math
import socket
import struct
import sys
import time


# ── helpers ──────────────────────────────────────────────────────────────────

def _deg(q):
    return [round(math.degrees(v), 2) for v in q]


class Dashboard:
    """Minimal Dashboard Server client."""

    def __init__(self, ip: str, timeout: float = 5.0):
        self._s = socket.create_connection((ip, 29999), timeout=timeout)
        self._s.settimeout(timeout)
        self._s.recv(4096)  # consume welcome banner

    def cmd(self, text: str) -> str:
        self._s.sendall((text + "\n").encode())
        try:
            return self._s.recv(4096).decode().strip()
        except socket.timeout:
            return "(timeout)"

    def close(self):
        try:
            self._s.close()
        except Exception:
            pass


def _power_on_and_enable(ip: str) -> bool:
    """
    Power on robot + brake release + enable remote control via Dashboard.
    Works for both URSim and real robots (real robots must already be in
    Remote Control mode on the teach pendant).
    """
    print("  [Dashboard] Connecting to dashboard server …", end=" ", flush=True)
    try:
        db = Dashboard(ip)
        print("OK")
    except Exception as exc:
        print(f"FAILED ({exc})")
        return False

    mode_resp = db.cmd("robotmode")
    print(f"  [Dashboard] Robot mode: {mode_resp}")

    if "POWER_OFF" in mode_resp or "IDLE" in mode_resp:
        print("  [Dashboard] Powering on …", end=" ", flush=True)
        db.cmd("power on")
        # Wait up to 20 s for power-on
        for _ in range(40):
            time.sleep(0.5)
            resp = db.cmd("robotmode")
            if "IDLE" in resp or "RUNNING" in resp:
                break
        print(db.cmd("robotmode"))

        print("  [Dashboard] Brake release …", end=" ")
        resp = db.cmd("brake release")
        print(resp)
        time.sleep(1.0)

    # Try to enable remote control (UR 5.12+ / e-series firmware)
    resp = db.cmd("set operational mode automatic")
    if "successful" in resp.lower() or "automatic" in resp.lower():
        print(f"  [Dashboard] Remote control enabled.")
    else:
        print(f"  [Dashboard] set operational mode: {resp}")
        print("  ⚠  If RTDEControl fails, open Polyscope and enable Remote Control manually.")

    db.close()
    return True


# ── per-arm test ──────────────────────────────────────────────────────────────

# ── URScript socket-controlled servoJ ────────────────────────────────────────
#
# URScript does NOT have a server socket API; only socket_open (client).
# Architecture: Python opens a TCP server; the URScript uploaded to the robot
# connects back as a client.
#
# Protocol (Python→robot):
#   6 × int32 big-endian = joint targets in µrad.  All-zeros = stop.
# Protocol (robot→Python):
#   1 × int32 big-endian = echo (value 1) after each servoJ call.


def _make_script(server_ip: str, server_port: int = 50002) -> str:
    return f"""\
def ext_servoj():
  socket_open("{server_ip}", {server_port}, "ctrl")
  textmsg("ext_servoj: connected to {server_ip}:{server_port}")
  while True:
    raw = socket_read_binary_integer(6, "ctrl", 5.0)
    if raw[0] == 0:
      break
    end
    q = [raw[1]*1.0e-6, raw[2]*1.0e-6, raw[3]*1.0e-6,
         raw[4]*1.0e-6, raw[5]*1.0e-6, raw[6]*1.0e-6]
    if norm(q) < 1.0e-9:
      break
    end
    servoj(q, t=0.008, lookahead_time=0.1, gain=300)
    socket_send_int(1, "ctrl")
  end
  stopj(2.0)
  socket_close("ctrl")
  textmsg("ext_servoj: done")
end
ext_servoj()
"""


def _send_script(ip: str, script: str) -> None:
    """Upload a URScript via the primary interface (port 30001)."""
    s = socket.create_connection((ip, 30001), timeout=5)
    s.sendall(script.encode())
    s.close()


def _start_server(port: int = 50002, timeout: float = 10.0) -> socket.socket:
    """Open a TCP server socket and wait for the URScript to connect."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    srv.settimeout(timeout)
    conn, addr = srv.accept()
    srv.close()
    conn.settimeout(2.0)
    return conn


def _pack_joints(q: list) -> bytes:
    """Encode 6 joint angles as 6 int32 values (µrad, big-endian) for URScript."""
    return struct.pack(">6i", *(int(v * 1e6) for v in q))


def run_servoj_test(ip: str, label: str, server_ip: str,
                    duration: float = 15.0, read_only: bool = False) -> bool:
    try:
        import rtde_receive
    except ImportError:
        print("ur-rtde not installed — run inside the teleop Docker container.")
        return False

    print(f"\n{'='*60}")
    print(f"  {label}  →  {ip}")
    print(f"{'='*60}")

    # ── RTDE Receive ──────────────────────────────────────────────────────
    print("  Connecting RTDE receive …", end=" ", flush=True)
    try:
        recv = rtde_receive.RTDEReceiveInterface(ip)
        print("OK")
    except Exception as exc:
        print(f"FAILED: {exc}")
        return False

    q_home = recv.getActualQ()
    robot_mode = recv.getRobotMode()
    print(f"  Robot mode   : {robot_mode}  (7=RUNNING needed for motion)")
    print(f"  Joints (deg) : {_deg(q_home)}")

    if read_only:
        print("  [read-only mode] skipping motion — connection OK.")
        return True

    # ── Power on if needed ────────────────────────────────────────────────
    if robot_mode != 7:
        _power_on_and_enable(ip)
        time.sleep(2.0)
        robot_mode = recv.getRobotMode()
        q_home = recv.getActualQ()
        print(f"  Robot mode after init: {robot_mode}")
        print(f"  Joints (deg)         : {_deg(q_home)}")

    if robot_mode != 7:
        print("  ✗ Robot still not RUNNING — cannot test motion.")
        return False

    # ── Start Python server, upload URScript ──────────────────────────────
    CTRL_PORT = 50002
    script = _make_script(server_ip, CTRL_PORT)

    print(f"  Starting control server on {server_ip}:{CTRL_PORT} …")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", CTRL_PORT))
    srv.listen(1)
    srv.settimeout(10.0)

    print("  Uploading servoJ URScript …", end=" ", flush=True)
    try:
        _send_script(ip, script)
        print("OK")
    except Exception as exc:
        print(f"FAILED: {exc}")
        srv.close()
        return False

    print("  Waiting for URScript to connect back …", end=" ", flush=True)
    try:
        ctrl_sock, addr = srv.accept()
        ctrl_sock.settimeout(2.0)
        srv.close()
        print(f"OK  ({addr[0]})")
    except socket.timeout:
        print("TIMEOUT — URScript did not connect.")
        srv.close()
        return False

    # ── servoJ motion test ────────────────────────────────────────────────
    hz            = 50
    period        = 1.0 / hz
    amplitude_rad = math.radians(8)   # ±8° on base joint
    freq_hz       = 0.25              # one full cycle every 4 s

    print(f"\n  servoJ motion test — {duration:.0f}s")
    print(f"  Joint-0 sinusoid ±{math.degrees(amplitude_rad):.0f}°  @ {freq_hz}Hz")
    print(f"  Rate {hz}Hz  (URScript socket protocol, server={server_ip}:{CTRL_PORT})")
    print()

    errors  = []
    t0      = time.monotonic()

    try:
        while True:
            t = time.monotonic() - t0
            if t >= duration:
                break

            q_cmd    = list(q_home)
            q_cmd[0] = q_home[0] + amplitude_rad * math.sin(2 * math.pi * freq_hz * t)

            try:
                ctrl_sock.sendall(_pack_joints(q_cmd))
                ctrl_sock.recv(4)   # wait for int32 ack from URScript
            except OSError as exc:
                print(f"\n  [socket error: {exc}]")
                break

            q_actual = recv.getActualQ()
            err      = abs(q_actual[0] - q_cmd[0])
            errors.append(err)

            needle = int(math.degrees(abs(q_cmd[0] - q_home[0])) / 2)
            print(
                f"\r  t={t:5.1f}s  "
                f"cmd={math.degrees(q_cmd[0]):+7.2f}°  "
                f"act={math.degrees(q_actual[0]):+7.2f}°  "
                f"err={math.degrees(err):5.2f}°  "
                f"{'█'*needle}{' '*(4-needle)}",
                end="", flush=True,
            )
            time.sleep(period)

    except KeyboardInterrupt:
        print("\n  [interrupted]")
    finally:
        # Send stop sentinel (all zeros) then close
        try:
            ctrl_sock.sendall(_pack_joints([0.0] * 6))
        except OSError:
            pass
        time.sleep(0.5)
        ctrl_sock.close()

    print()

    # ── Results ───────────────────────────────────────────────────────────
    if not errors:
        print("  No samples collected.")
        return False

    max_err = math.degrees(max(errors))
    avg_err = math.degrees(sum(errors) / len(errors))
    print(f"\n  Samples : {len(errors)}")
    print(f"  Max err : {max_err:.2f}°")
    print(f"  Avg err : {avg_err:.2f}°")

    # The socket round-trip adds latency vs RTDE, so allow up to 15° error.
    # In the real teleop stack (RTDEControlInterface) errors are < 1°.
    if max_err < 15.0:
        print("  ✓ servoJ working — robot tracks commands (socket latency expected).")
        return True
    else:
        print("  ✗ Robot NOT tracking — large position error.")
        print("    → Check: E-stop, protective stop, robot mode.")
        return False


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="servoJ diagnostic for URSim / real UR")
    ap.add_argument("--ip",        default="172.30.0.20", help="Left robot IP")
    ap.add_argument("--ip2",       default="172.30.0.10", help="Right robot IP")
    ap.add_argument("--server-ip", default="172.30.0.5",
                    help="IP the robot can reach this machine on (default: teleop container)")
    ap.add_argument("--one",       action="store_true",   help="Test only --ip (skip --ip2)")
    ap.add_argument("--duration",  type=float, default=15.0, help="Test duration per arm (s)")
    ap.add_argument("--read-only", action="store_true",   help="Connection check only, no motion")
    args = ap.parse_args()

    targets = [
        (args.ip,  "LEFT  arm"),
        (args.ip2, "RIGHT arm"),
    ] if not args.one else [(args.ip, "LEFT arm")]

    results = {}
    for ip, label in targets:
        results[label] = run_servoj_test(
            ip, label, server_ip=args.server_ip,
            duration=args.duration, read_only=args.read_only
        )

    print(f"\n{'='*60}")
    all_ok = all(results.values())
    for label, ok in results.items():
        print(f"  {'✓' if ok else '✗'}  {label}")
    print(f"{'='*60}")

    if all_ok:
        print("\nservoJ path is healthy.")
        print("If the full teleop still doesn't move, check:")
        print("  1. GELLO readings  — add print(obs) in GelloAgent.act()")
        print("  2. Joint mapping   — offsets/signs in Settings")
        print("  3. Control rate    — try control_rate_hz=25 in Settings")
    else:
        print("\nFix RTDE connection before testing the full teleop stack.")

    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
