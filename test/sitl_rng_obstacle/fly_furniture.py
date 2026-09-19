#!/usr/bin/env python3
"""
Fly the x500_flow_tof quadcopter over indoor furniture in Gazebo SITL and
record a ulog for the range-finder height-reference comparison.

The vehicle uses optical flow plus a downward ToF range finder (no GPS unless
--gps is given) with the range finder as EKF2 height reference
(EKF2_HGT_REF=2, EKF2_RNG_CTRL=2).  The run is otherwise identical for
--obst 0 (terrain following) and --obst 1 (hold altitude over obstacles),
so the two logs can be compared with analyze.py.

Sequence
  1. kill stale px4 / gz server processes
  2. start the Gazebo server headless with worlds/indoor_furniture.sdf
  3. spawn models/x500_flow_tof at the clear spawn point in the living room
  4. start PX4 (build/px4_sitl_default) attached to that model, with the
     EKF2 / MPC parameters passed as PX4_PARAM_* environment variables
  5. MAVSDK: wait for a valid local position, start Offboard, arm, climb,
     fly the route (slowly over the step, fast passes over the dining table
     and the couch, a slow pass over the coffee table, a 4 s hover above the
     dining table; with --doors also through two doorways and over the bed),
     return, land, disarm
  6. shut PX4 and Gazebo down and copy the newest .ulg into --out

Usage
  fly_furniture.py --obst 0            # baseline, terrain following
  fly_furniture.py --obst 1            # hold altitude over obstacles
  fly_furniture.py --obst 1 --gps      # same, with GPS fused as well
  fly_furniture.py --obst 1 --doors    # also through the doorways and over the bed
"""

import argparse
import asyncio
import glob
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

HERE = os.path.dirname(os.path.abspath(__file__))
PX4_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
BUILD_DIR = os.path.join(PX4_ROOT, "build", "px4_sitl_default")
ROOTFS = os.path.join(BUILD_DIR, "rootfs")
PX4_BIN = os.path.join(BUILD_DIR, "bin", "px4")
GZ_ENV = os.path.join(ROOTFS, "gz_env.sh")

WORLD_NAME = "indoor_furniture"
WORLD_SDF = os.path.join(HERE, "worlds", f"{WORLD_NAME}.sdf")
MODELS_DIR = os.path.join(HERE, "models")
MODEL_SDF = os.path.join(MODELS_DIR, "x500_flow_tof", "model.sdf")
MODEL_NAME = "x500_flow_tof_0"

# Spawn point in Gazebo world coordinates (x east, y north).  Faces north
# (yaw pi/2 in ENU) so that the PX4 heading is ~0.
SPAWN_XY = (-8.5, 1.5)
SPAWN_YAW = math.pi / 2

WP_REACHED_M = 0.3      # advance to the next leg when within this distance ...
WP_SETTLED_M_S = 0.4    # ... and slower than this (no corner-cutting at speed)
ACCEL_M_S2 = 1.5        # carrot acceleration/deceleration along a leg
ABORT_DIST_M = 4.0      # abort the route if the estimate is this far from the carrot
ABORT_DRIFT_M = 2.5     # ... or if the estimate is this far from the ground truth
SETPOINT_HZ = 20.0
SLOW_SPEED = 1.0        # m/s for the coffee-table pass
HOVER_S = 4.0           # hover time above the dining table


def route(speed, doors=False):
    """Flight route as (label, gz_x, gz_y, speed, hover_s) legs.

    All positions are Gazebo world coordinates; the altitude is --alt.
    Furniture (see worlds/indoor_furniture.sdf):
      dining table  x[-7.3,-5.7] y[3.05,3.95]   coffee table x[-3.5,-2.5] y[3.2,3.8]
      couch         x[-4,-2]     y[4.55,5.55]   step         x[-5.5,-3.5] y[0.5,2.5]
      bed           x[5.75,7.75] y[2.2,3.8]     doors at x=0 and x=3, y in [2.0,4.0], 2.3 m high

    The default route stays in the living room.  Without GPS every obstacle
    crossed costs some position drift (the flow is scaled with the wrong
    distance while the range finder sees the obstacle, and the tracker is
    disturbed at height discontinuities), and the estimate was repeatedly
    lost in the doorways, so the hallway/bedroom part is optional (--doors).
    """
    legs = [
        # The step is crossed (and left) slowly: crossing its edge at 3 m/s
        # produced ~1 m of position drift and a crash into the west wall.
        ("SLOW over step platform", -4.5, 1.5, SLOW_SPEED, 0.0),
    ]
    if doors:
        legs += [
            ("to living-room door", -2.5, 3.0, SLOW_SPEED, 0.0),
            ("SLOW through door into hallway", 1.5, 3.0, SLOW_SPEED, 0.0),
            ("SLOW through door into bedroom", 4.2, 3.0, SLOW_SPEED, 0.0),
            ("FAST pass over bed", 8.3, 3.0, speed, 0.0),
            ("FAST pass back over bed", 4.2, 3.0, speed, 0.0),
            ("SLOW back through hallway door", 1.5, 3.0, SLOW_SPEED, 0.0),
            ("SLOW back into living room", -2.5, 3.0, SLOW_SPEED, 0.0),
            ("SLOW back over step platform", -4.5, 1.5, SLOW_SPEED, 0.0),
        ]
    legs += [
        ("SLOW off the step platform", -6.5, 1.5, SLOW_SPEED, 0.0),
        ("to west wall", -8.6, 1.5, speed, 0.0),
        ("to dining-table run-in", -8.6, 3.5, speed, 0.0),
        ("FAST pass over dining table", -4.5, 3.5, speed, 0.0),
        ("SLOW pass over coffee table", -1.8, 3.5, SLOW_SPEED, 0.0),
        ("to couch run-in", -1.5, 5.0, speed, 0.0),
        ("FAST pass over couch", -5.0, 5.0, speed, 0.0),
        ("HOVER above dining table", -6.5, 3.5, speed, HOVER_S),
        # Slow leg off the table: leaving it leaves dist_bottom wrong for
        # ~1.5 s, which mis-scales the flow velocity; at 1 m/s the position
        # error stays small (at 3 m/s the vehicle overshot by 2.5 m).
        ("SLOW leg off the dining table", -8.6, 3.5, SLOW_SPEED, 0.0),
        ("return to spawn", SPAWN_XY[0], SPAWN_XY[1], SLOW_SPEED, 0.0),
    ]
    return legs


# --------------------------------------------------------------------------
# process helpers
# --------------------------------------------------------------------------

def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def ancestors():
    """This process and every ancestor (a shell whose command line mentions
    e.g. mavsdk_server would otherwise match the kill patterns)."""
    pids = set()
    pid = os.getpid()
    while pid > 1:
        pids.add(pid)
        try:
            with open(f"/proc/{pid}/stat") as f:
                pid = int(f.read().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return pids


def kill_stale():
    """Kill leftover px4, gz server and mavsdk_server processes (not this script).

    A stale mavsdk_server is fatal: it keeps udp/14540 and the gRPC port, so
    the new MAVSDK client silently attaches to it and never sees the new PX4."""
    keep = ancestors()  # never kill this script or the shells that started it
    victims = set()
    for pattern in (r"^gz sim", r"^ruby.*gz sim", r"/bin/px4( |$)", r"^px4( |$)", r"mavsdk_server"):
        out = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True).stdout
        victims.update(int(p) for p in out.split() if int(p) not in keep)
    for pid in victims:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    if victims:
        log(f"killed stale processes: {sorted(victims)}")
        time.sleep(2.0)
        for pid in victims:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def kill_mavsdk_server():
    out = subprocess.run(["pgrep", "-f", "mavsdk_server"], capture_output=True, text=True).stdout
    keep = ancestors()
    for pid in (int(p) for p in out.split()):
        if pid not in keep:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass


def gz_environment():
    """Environment for gz and PX4: gz_env.sh plus our models/worlds directory."""
    if not os.path.exists(GZ_ENV):
        sys.exit(f"{GZ_ENV} not found: build px4_sitl_default first")
    dump = subprocess.run(["bash", "-c", f"source '{GZ_ENV}' && env -0"],
                          capture_output=True, check=True).stdout
    env = dict(os.environ)
    for item in dump.split(b"\0"):
        if b"=" in item:
            k, v = item.decode(errors="replace").split("=", 1)
            env[k] = v
    env["GZ_SIM_RESOURCE_PATH"] = f"{MODELS_DIR}:{os.path.dirname(WORLD_SDF)}:" + env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_IP"] = "127.0.0.1"
    return env


def gz_topics(env):
    out = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, env=env, timeout=20)
    return out.stdout.splitlines()


def wait_for(predicate, timeout, what, period=1.0):
    """Poll predicate() until true; raise TimeoutError otherwise."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            if predicate():
                return
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            pass
        time.sleep(period)
    raise TimeoutError(f"timed out after {timeout:.0f}s waiting for {what}")


def start_gazebo(env, out_dir, render_engine):
    cmd = ["gz", "sim", "-r", "-s", "--verbose=1"]
    if render_engine:
        cmd += ["--render-engine", render_engine]
    cmd.append(WORLD_SDF)
    log("starting gz server: " + " ".join(cmd))
    gz_log = open(os.path.join(out_dir, "gz.log"), "w")
    proc = subprocess.Popen(cmd, env=env, stdout=gz_log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)
    wait_for(lambda: f"/world/{WORLD_NAME}/clock" in gz_topics(env), 60, "gz world clock topic")
    log(f"gz world '{WORLD_NAME}' is up")
    return proc


def spawn_model(env):
    x, y = SPAWN_XY
    sdf = (f"<sdf version='1.6'><include><uri>file://{MODEL_SDF}</uri>"
           f"<pose>{x} {y} 0.3 0 0 {SPAWN_YAW}</pose></include></sdf>")
    req = f'name: "{MODEL_NAME}", allow_renaming: false, sdf: "{sdf}"'
    r = subprocess.run(["gz", "service", "-s", f"/world/{WORLD_NAME}/create",
                        "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
                        "--timeout", "5000", "--req", req],
                       capture_output=True, text=True, env=env, timeout=30)
    if "data: true" not in r.stdout:
        raise RuntimeError(f"model spawn failed: {r.stdout} {r.stderr}")
    lidar = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/lidar_sensor_link/sensor/lidar/scan"
    wait_for(lambda: lidar in gz_topics(env), 30, "lidar topic of the spawned model")
    log(f"spawned {MODEL_NAME} at {SPAWN_XY}")


def px4_params(args):
    """PX4 parameters for this run, applied via PX4_PARAM_* at startup."""
    p = {
        "EKF2_HGT_REF": 2,          # range finder is the height reference
        "EKF2_RNG_CTRL": 2,         # always fuse the range finder
        "EKF2_RNG_OBST": args.obst,  # feature under test (ignored if unknown to the binary)
        "EKF2_RNG_NOISE": 0.03,
        "EKF2_RNG_SFE": 0.0,
        "EKF2_OF_CTRL": 1,
        # the PX4 shell readbacks pause the offboard setpoint stream for up to a
        # few seconds; do not let the offboard-loss failsafe land the vehicle
        "COM_OF_LOSS_T": 5.0,
        # the carrot moves at --speed; +0.3 m/s lets the position loop catch up
        "MPC_XY_VEL_MAX": args.speed + 0.3,
        "MPC_XY_CRUISE": args.speed,
        "MPC_VEL_MANUAL": min(args.speed, 10.0),
        # Gentle manoeuvring: the simulated flow sensor's velocity error grows
        # with angular rate (rotation compensation), and every m/s of velocity
        # error is a metre of drift that cannot be corrected without GPS.
        "MPC_TILTMAX_AIR": 20.0,
        "MPC_ACC_HOR_MAX": 2.0,
        "MPC_ACC_HOR": 2.0,
        # Land at 0.7 m/s so that the landing phase is short; nothing in the
        # flight depends on the landing speed.
        "MPC_LAND_SPEED": 0.7,
    }
    # GPS is set explicitly in both modes: PX4_PARAM_* values are stored in
    # rootfs/fs/parameters.bson and survive into the next run, whereas the
    # airframe file only uses `param set-default`, which does not override a
    # stored value. airframe 4021 disables GPS; SIM_GPS_USED is the simulated
    # satellite count (the gz bridge reports a 3D fix from 4 satellites).
    if args.gps:
        p.update({"SYS_HAS_GPS": 1, "SIM_GPS_USED": 10, "EKF2_GPS_CTRL": 7})
    else:
        p.update({"SYS_HAS_GPS": 0, "SIM_GPS_USED": 0, "EKF2_GPS_CTRL": 0})
    return p


def start_px4(env, args, out_dir):
    params = px4_params(args)
    px4_env = dict(env)
    px4_env.update({
        "PX4_SIM_MODEL": "gz_x500_flow",
        "PX4_GZ_MODEL_NAME": MODEL_NAME,
        "PX4_GZ_WORLD": WORLD_NAME,
        "HEADLESS": "1",
    })
    for k, v in params.items():
        px4_env[f"PX4_PARAM_{k}"] = str(v)
    # start from the airframe defaults: drop parameters stored by earlier runs
    for name in ("parameters.bson", "parameters_backup.bson"):
        path = os.path.join(ROOTFS, "fs", name)
        if os.path.exists(path):
            os.remove(path)
            log(f"removed stored parameters {path}")
    log("starting PX4 with params " + " ".join(f"{k}={v}" for k, v in params.items()))
    px4_log = open(os.path.join(out_dir, "px4.log"), "w")
    # stdin is a pipe so that we can send 'shutdown' at the end
    proc = subprocess.Popen([PX4_BIN], cwd=ROOTFS, env=px4_env, stdin=subprocess.PIPE,
                            stdout=px4_log, stderr=subprocess.STDOUT, start_new_session=True)
    proc.log_path = px4_log.name  # used by px4_shell() to read command output

    # Only hand over to MAVSDK once the PX4 startup script has finished.  If
    # mavsdk_server is started while PX4 is still booting, its telemetry
    # streams (health etc.) were observed to never deliver a sample.
    def started():
        if proc.poll() is not None:
            raise RuntimeError(f"PX4 exited during startup (code {proc.returncode}); see px4.log")
        with open(px4_log.name, errors="replace") as f:
            return "Startup script returned successfully" in f.read()
    wait_for(started, 90, "PX4 startup script")
    time.sleep(2.0)
    log("PX4 started")
    return proc, params


def stop_px4(proc):
    if proc.poll() is not None:
        return
    log("shutting PX4 down")
    try:
        proc.stdin.write(b"shutdown\n")
        proc.stdin.flush()
        proc.wait(timeout=15)
        return
    except (OSError, subprocess.TimeoutExpired):
        pass
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


def px4_shell(proc, command, wait_s=1.0):
    """Run a command in the PX4 shell (via its stdin) and return its output.

    The output is what PX4 appended to px4.log after the command was sent."""
    log_path = getattr(proc, "log_path", None)
    if proc.poll() is not None or log_path is None:
        return ""
    with open(log_path, "rb") as f:
        f.seek(0, os.SEEK_END)
        offset = f.tell()
    try:
        proc.stdin.write((command + "\n").encode())
        proc.stdin.flush()
    except OSError:
        return ""
    time.sleep(wait_s)
    with open(log_path, "rb") as f:
        f.seek(offset)
        return f.read().decode(errors="replace")


def px4_field(text, name):
    """Value of '<name>: <value>' in listener output, or None."""
    m = re.search(rf"^\s*{re.escape(name)}:\s*(\S+)", text, re.M)
    return m.group(1) if m else None


def check_sensors(px4_proc):
    """Read flow quality, EKF fusion flags and dist_bottom through the PX4 shell."""
    flow = px4_shell(px4_proc, "listener sensor_optical_flow 1")
    flags = px4_shell(px4_proc, "listener estimator_status_flags 1")
    lpos = px4_shell(px4_proc, "listener vehicle_local_position 1")
    q = px4_field(flow, "quality")
    cs_flow = px4_field(flags, "cs_opt_flow")
    cs_rng = px4_field(flags, "cs_rng_hgt")
    cs_gps = px4_field(flags, "cs_gnss_pos")
    try:
        dist_bottom = float(px4_field(lpos, "dist_bottom"))
    except (TypeError, ValueError):
        dist_bottom = None
    log(f"sensors: flow quality={q} cs_opt_flow={cs_flow} cs_rng_hgt={cs_rng} cs_gnss_pos={cs_gps} dist_bottom={dist_bottom}")
    return q, cs_flow, cs_rng, cs_gps, dist_bottom


def stop_gazebo(proc):
    if proc.poll() is not None:
        return
    log("stopping gz server")
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()


def newest_ulg():
    """Newest ulog written by this SITL build (logger writes to rootfs/fs/log)."""
    files = (glob.glob(os.path.join(ROOTFS, "fs", "log", "*", "*.ulg")) +
             glob.glob(os.path.join(ROOTFS, "log", "*", "*.ulg")))
    return max(files, key=os.path.getmtime) if files else None


# --------------------------------------------------------------------------
# flight
# --------------------------------------------------------------------------

async def first(stream, timeout, what):
    """First sample of a MAVSDK telemetry stream, or TimeoutError."""
    async def _get():
        async for sample in stream:
            return sample
    try:
        return await asyncio.wait_for(_get(), timeout)
    except asyncio.TimeoutError:
        raise TimeoutError(f"no {what} sample within {timeout:.0f} s")


class Setpoints:
    """Streams the current Offboard setpoint at SETPOINT_HZ until stopped."""

    def __init__(self, drone, yaw_deg):
        self.drone = drone
        self.yaw = yaw_deg
        self.pos = PositionNedYaw(0.0, 0.0, 0.0, yaw_deg)
        self.vel = VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
        self._task = None

    def set(self, n, e, d, vn=0.0, ve=0.0):
        self.pos = PositionNedYaw(n, e, d, self.yaw)
        self.vel = VelocityNedYaw(vn, ve, 0.0, self.yaw)

    async def _run(self):
        while True:
            await self.drone.offboard.set_position_velocity_ned(self.pos, self.vel)
            await asyncio.sleep(1.0 / SETPOINT_HZ)

    def start(self):
        self._task = asyncio.ensure_future(self._run())

    async def stop(self):
        if self._task:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None


class Flight:
    def __init__(self, drone, args, px4_proc=None):
        self.drone = drone
        self.args = args
        self.px4_proc = px4_proc
        self.pos = None       # latest PositionVelocityNed
        self.armed = False
        self.in_air = False
        self.health = None    # latest Health
        self.failed = None    # set when the route was aborted
        self.drift = 0.0      # |estimate - ground truth| horizontal, from the PX4 shell
        self.drift_abort = False
        self.n0 = self.e0 = 0.0  # local-position offset of the spawn point
        self.d_cruise = -args.alt  # NED z setpoint of the route (trimmed after the climb)
        self.sp = None

    async def track_telemetry(self):
        async def pos():
            async for p in self.drone.telemetry.position_velocity_ned():
                self.pos = p

        async def armed():
            async for a in self.drone.telemetry.armed():
                self.armed = a

        async def in_air():
            async for a in self.drone.telemetry.in_air():
                self.in_air = a

        async def health():
            async for h in self.drone.telemetry.health():
                self.health = h

        return [asyncio.ensure_future(c()) for c in (pos, armed, in_air, health)]

    async def track_drift(self):
        """Compare the estimate with vehicle_local_position_groundtruth (read
        through the PX4 shell, ~1 Hz).  Only a safety net: a vehicle that has
        drifted far from where it thinks it is will hit a wall, so land early
        instead of wrecking the log.  The estimate itself is never corrected."""
        while self.px4_proc:
            p = self.pos.position  # sampled when the listener command is issued
            text = await asyncio.to_thread(px4_shell, self.px4_proc, "listener vehicle_local_position_groundtruth 1", 0.4)
            try:
                n_true = float(px4_field(text, "x")) - SPAWN_XY[1]
                e_true = float(px4_field(text, "y")) - SPAWN_XY[0]
                self.drift = math.hypot(p.north_m - self.n0 - n_true, p.east_m - self.e0 - e_true)
                if self.drift > ABORT_DRIFT_M:
                    self.drift_abort = True
            except (TypeError, ValueError, AttributeError):
                pass
            await asyncio.sleep(0.5)

    def ned(self, gz_x, gz_y):
        """Gazebo world (x east, y north) -> local NED (n, e) relative to spawn."""
        return self.n0 + (gz_y - SPAWN_XY[1]), self.e0 + (gz_x - SPAWN_XY[0])

    def dist_to(self, n, e, d):
        p = self.pos.position
        return math.sqrt((p.north_m - n) ** 2 + (p.east_m - e) ** 2 + (p.down_m - d) ** 2)

    def speed(self):
        v = self.pos.velocity
        return math.sqrt(v.north_m_s ** 2 + v.east_m_s ** 2 + v.down_m_s ** 2)

    def check_abort(self, n, e, d):
        """Raise if the flight has clearly gone wrong (estimate diverged, failsafe)."""
        if not self.armed:
            raise RuntimeError("vehicle disarmed during the route")
        if self.drift_abort:
            raise RuntimeError(f"estimate {self.drift:.1f} m from ground truth: aborting route")
        if self.dist_to(n, e, d) > ABORT_DIST_M:
            raise RuntimeError(f"estimate {self.dist_to(n, e, d):.1f} m from setpoint: aborting route")

    async def wait_within(self, n, e, d, tol, timeout, what, settle=True):
        """Wait until the estimate is within tol of (n, e, d) and, if settle, nearly stopped."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.pos and self.dist_to(n, e, d) < tol and (not settle or self.speed() < WP_SETTLED_M_S):
                return True
            if self.pos:
                self.check_abort(n, e, d)
            await asyncio.sleep(0.05)
        log(f"WARNING: {what}: not within {tol} m after {timeout:.0f}s (dist {self.dist_to(n, e, d):.2f} m)")
        return False

    async def fly_leg(self, label, gz_x, gz_y, speed, hover_s):
        """Move a position+velocity 'carrot' along the straight line to the
        waypoint with a trapezoidal speed profile, then hold there."""
        d = self.d_cruise
        n1, e1 = self.ned(gz_x, gz_y)
        n0, e0 = self.sp.pos.north_m, self.sp.pos.east_m
        length = math.hypot(n1 - n0, e1 - e0)
        log(f"leg: {label} -> gz({gz_x:.1f},{gz_y:.1f}) {length:.1f} m at {speed:.1f} m/s (drift {self.drift:.2f} m)")
        if length > 1e-3:
            un, ue = (n1 - n0) / length, (e1 - e0) / length
            v_peak = min(speed, math.sqrt(length * ACCEL_M_S2))  # short legs never reach `speed`
            t_acc = v_peak / ACCEL_M_S2
            s_acc = 0.5 * ACCEL_M_S2 * t_acc ** 2
            t_cruise = (length - 2 * s_acc) / v_peak
            t_total = 2 * t_acc + t_cruise
            t0 = time.time()
            while True:
                t = time.time() - t0
                if t >= t_total:
                    break
                if t < t_acc:
                    v = ACCEL_M_S2 * t
                    s = 0.5 * ACCEL_M_S2 * t ** 2
                elif t < t_acc + t_cruise:
                    v = v_peak
                    s = s_acc + v_peak * (t - t_acc)
                else:
                    tr = t_total - t
                    v = ACCEL_M_S2 * tr
                    s = length - 0.5 * ACCEL_M_S2 * tr ** 2
                self.sp.set(n0 + un * s, e0 + ue * s, d, un * v, ue * v)
                self.check_abort(n0 + un * s, e0 + ue * s, d)
                await asyncio.sleep(1.0 / SETPOINT_HZ)
        self.sp.set(n1, e1, d)
        await self.wait_within(n1, e1, d, WP_REACHED_M, length / max(speed, 0.5) + 15.0, label)
        if hover_s > 0:
            log(f"hovering {hover_s:.0f} s")
            await asyncio.sleep(hover_s)

    async def run(self):
        drone, args = self.drone, self.args
        tele = await self.track_telemetry()

        # ----- wait for a usable local position estimate
        log("waiting for local position (flow + range finder)")
        t0 = time.time()
        while not (self.health and self.health.is_local_position_ok and self.pos is not None):
            if time.time() - t0 > 90:
                raise TimeoutError(f"no valid local position after 90 s (health={self.health})")
            await asyncio.sleep(1.0)
        if args.gps:
            log("waiting for GPS fix / global position")
            t0 = time.time()
            while not self.health.is_global_position_ok and time.time() - t0 < 60:
                await asyncio.sleep(1.0)
            if not self.health.is_global_position_ok:
                log("WARNING: global position never became OK; continuing anyway")
        # Hold the current yaw for the whole flight.  (telemetry.heading() is
        # not used: that stream never delivers a sample on this setup.)
        heading = await first(drone.telemetry.attitude_euler(), 10.0, "attitude")
        heading = heading.yaw_deg
        self.n0, self.e0 = self.pos.position.north_m, self.pos.position.east_m
        log(f"local position ok: n={self.n0:.2f} e={self.e0:.2f} d={self.pos.position.down_m:.2f} heading={heading:.1f} deg")

        # ----- offboard + arm + climb
        self.sp = Setpoints(drone, heading)
        self.sp.set(self.n0, self.e0, self.pos.position.down_m)
        self.sp.start()
        await asyncio.sleep(1.0)
        for attempt in range(10):
            try:
                await drone.offboard.start()
                break
            except OffboardError as err:
                log(f"offboard start failed ({err._result.result}), retrying")
                await asyncio.sleep(2.0)
        else:
            raise RuntimeError("could not start offboard mode")
        t0 = time.time()
        while not self.armed:
            try:
                await drone.action.arm()
            except Exception as err:  # ActionError
                log(f"arm failed ({err}), retrying")
            if time.time() - t0 > 60:
                raise RuntimeError("could not arm")
            await asyncio.sleep(1.0)
        log("armed, climbing to %.1f m" % args.alt)
        self.sp.set(self.n0, self.e0, -args.alt)
        await self.wait_within(self.n0, self.e0, -args.alt, 0.2, 40, "climb")
        await asyncio.sleep(2.0)  # settle
        if self.px4_proc:
            q, cs_flow, cs_rng, _, dist_bottom = await asyncio.to_thread(check_sensors, self.px4_proc)
            if cs_flow != "True" or cs_rng != "True":
                log("WARNING: optical flow or range height is not being fused; the route will probably fail")
            # Note: the EKF height creeps up by ~0.15 m between arming and
            # take-off (range finder not fused while on the ground), so at
            # the setpoint the range finder reads ~0.15 m less than --alt.
            # The analysis compares against ground truth, so this is not
            # corrected here (--alt-trim adds a fixed offset if needed).
            self.d_cruise = -(args.alt + args.alt_trim)
            if abs(args.alt_trim) > 1e-3:
                log(f"cruise z setpoint {self.d_cruise:.2f} (--alt-trim {args.alt_trim:+.2f})")
                self.sp.set(self.n0, self.e0, self.d_cruise)
                await self.wait_within(self.n0, self.e0, self.d_cruise, 0.15, 15, "altitude trim")
        log("at cruise altitude, starting route")
        drift_task = asyncio.ensure_future(self.track_drift()) if self.px4_proc else None

        # ----- route
        try:
            for leg in route(args.speed, args.doors):
                await self.fly_leg(*leg)
            log("route complete, landing")
        except RuntimeError as err:
            log(f"ERROR: {err}; landing now")
            self.failed = str(err)

        # ----- land
        if drift_task:
            drift_task.cancel()
        await drone.action.land()
        await self.sp.stop()
        t0 = time.time()
        while self.armed and time.time() - t0 < 45:
            await asyncio.sleep(0.5)
        if self.armed:
            log("WARNING: still armed 45 s after land command; forcing disarm")
            if self.px4_proc:
                px4_shell(self.px4_proc, "commander disarm -f", 2.0)
            if self.armed:
                try:
                    await drone.action.kill()
                except Exception as err:
                    log(f"kill failed: {err}")
        log("landed and disarmed")
        for t in tele:
            t.cancel()


async def fly(args, px4_proc):
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    log("waiting for PX4 MAVLink connection")
    t0 = time.time()
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
        if time.time() - t0 > 90:
            raise TimeoutError("no MAVLink connection after 90 s")
    log("connected")
    flight = Flight(drone, args, px4_proc)
    await flight.run()
    if flight.failed:
        raise RuntimeError(flight.failed)


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--obst", type=int, choices=(0, 1), required=True, help="EKF2_RNG_OBST value")
    ap.add_argument("--gps", action="store_true", help="also fuse (simulated) GPS")
    ap.add_argument("--speed", type=float, default=3.0, help="cruise speed for the fast passes [m/s]")
    ap.add_argument("--alt", type=float, default=1.5, help="flight altitude above the spawn floor [m]")
    ap.add_argument("--alt-trim", type=float, default=0.0,
                    help="extra height added to --alt after the climb [m] (e.g. 0.2 to compensate the EKF pre-takeoff creep)")
    ap.add_argument("--out", default=None, help="output directory (default: runs/<timestamp>_obst<N>[_gps])")
    ap.add_argument("--doors", action="store_true",
                    help="also fly through the hallway doors and over the bed (estimate often lost there)")
    ap.add_argument("--render-engine", default=None,
                    help="gz render engine override (e.g. ogre) if ogre2 sensors do not render headless")
    ap.add_argument("--keep", action="store_true", help="leave PX4 and gz running after the flight")
    args = ap.parse_args()

    if args.out is None:
        tag = f"{time.strftime('%Y%m%d_%H%M%S')}_obst{args.obst}" + ("_gps" if args.gps else "")
        args.out = os.path.join(HERE, "runs", tag)
    os.makedirs(args.out, exist_ok=True)
    log(f"output directory: {args.out}")

    kill_stale()
    env = gz_environment()
    gz_proc = px4_proc = None
    params = {}
    status = "failed"
    t_start = time.time()
    ulg_before = newest_ulg()
    try:
        gz_proc = start_gazebo(env, args.out, args.render_engine)
        spawn_model(env)
        px4_proc, params = start_px4(env, args, args.out)
        asyncio.run(asyncio.wait_for(fly(args, px4_proc), timeout=300))
        status = "ok"
    except Exception as err:
        log(f"ERROR: {err!r}")
    finally:
        if args.keep and status == "ok":
            log("--keep: leaving PX4 and gz running")
        else:
            if px4_proc:
                stop_px4(px4_proc)
            if gz_proc:
                stop_gazebo(gz_proc)
    # ----- collect the log
    ulg = newest_ulg()
    copied = None
    if ulg and ulg != ulg_before:
        copied = os.path.join(args.out, os.path.basename(ulg))
        shutil.copy2(ulg, copied)
        log(f"log copied to {copied}")
    else:
        log("WARNING: no new .ulg found under " + os.path.join(ROOTFS, "fs", "log"))
    with open(os.path.join(args.out, "run_info.json"), "w") as f:
        json.dump({"status": status, "args": vars(args), "px4_params": params,
                   "ulg": os.path.basename(copied) if copied else None,
                   "wall_time_s": round(time.time() - t_start, 1)}, f, indent=2)
    log(f"done ({status}) in {time.time() - t_start:.0f} s")
    # After a failed flight the MAVSDK gRPC threads and the mavsdk_server it
    # started keep the interpreter alive; make sure both go away.
    kill_mavsdk_server()
    sys.stdout.flush()
    os._exit(0 if status == "ok" else 1)


if __name__ == "__main__":
    main()
