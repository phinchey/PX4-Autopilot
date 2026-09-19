#!/usr/bin/env python3
"""Film a logged flight: replay the Gazebo ground truth of a .ulg in the
indoor_furniture world and record it with the flight_cameras model.

    replay_video.py <flight.ulg> --out <dir> [--start S] [--end S]

Why not record during the flight (fly_furniture.py --video)?  Rendering and
encoding two extra cameras slows Gazebo to a third of real time, and the
simulated optical flow degrades noticeably when the simulation runs that
slowly; every filmed no-GPS flight drifted into an abort.  Replaying the
ground truth afterwards costs the flight nothing: the world runs without
PX4, the vehicle model is static and its pose is set from
vehicle_local_position_groundtruth / vehicle_attitude_groundtruth at the
current simulation time (read from the world statistics topic), and the
CameraVideoRecorder (sim-time timestamps) produces the videos, so a slow
render only makes the replay take longer.  Propellers do not spin.

Output: wide.mp4, side.mp4 and the combined flight.mp4 (side view with the
corner view as an inset) in --out.
"""

import argparse
import math
import os
import sys
import time

import numpy as np
from pyulog import ULog

sys.path.append("/usr/lib/python3/dist-packages")  # gz python bindings (system packages)
from gz.transport13 import Node  # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean  # noqa: E402
from gz.msgs10.entity_factory_pb2 import EntityFactory  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.msgs10.video_record_pb2 import VideoRecord  # noqa: E402
from gz.msgs10.world_stats_pb2 import WorldStatistics  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fly_furniture as ff  # noqa: E402  (gz_environment, start/stop_gazebo, combine_video, paths)


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def load_trajectory(ulg_path):
    """Ground truth pose in the Gazebo world frame (x east, y north, z up; roll,
    pitch, yaw ENU/FLU), sampled at the ground truth rate."""
    u = ULog(ulg_path, message_name_filter_list=["vehicle_local_position_groundtruth",
                                                  "vehicle_attitude_groundtruth"])
    data = {d.name: d.data for d in u.data_list}
    p = data["vehicle_local_position_groundtruth"]
    a = data["vehicle_attitude_groundtruth"]
    t = p["timestamp"] / 1e6
    # NED (north, east, down) -> gz (east, north, up); the ground truth is
    # already in the world frame (the spawn point appears as its own coordinates)
    x, y, z = p["y"], p["x"], -p["z"]
    ta = a["timestamp"] / 1e6
    q = np.stack([np.interp(t, ta, a[f"q[{i}]"]) for i in range(4)], axis=1)
    w, qx, qy, qz = q.T
    # NED/FRD quaternion -> roll, pitch, yaw (NED)
    roll = np.arctan2(2 * (w * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2))
    pitch = np.arcsin(np.clip(2 * (w * qy - qz * qx), -1, 1))
    yaw = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))
    # NED/FRD -> ENU/FLU: same roll, pitch and yaw flip sign, yaw offset pi/2
    return t - t[0], x, y, z, roll, -pitch, math.pi / 2 - yaw


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


class Replay:
    def __init__(self, world):
        self.node = Node()
        self.world = world
        self.clock = ff.SimClock()
        if not self.node.subscribe(WorldStatistics, f"/world/{world}/stats", self._on_stats):
            raise RuntimeError("could not subscribe to the world statistics")

    def _on_stats(self, msg):
        self.clock.sample(msg.sim_time.sec + msg.sim_time.nsec * 1e-9)

    def request(self, service, req, rep_type=Boolean, timeout_ms=5000, attempts=3):
        for attempt in range(attempts):
            ok, rep = self.node.request(service, req, type(req), rep_type, timeout_ms)
            if ok and (rep_type is not Boolean or rep.data):
                return rep
            if attempt + 1 < attempts:
                time.sleep(1.0)  # service discovery may still be in progress
        raise RuntimeError(f"service {service} failed: ok={ok} reply={rep}")

    def spawn(self, name, sdf_path, pose, static=False):
        req = EntityFactory()
        req.name = name
        req.allow_renaming = False
        st = "<static>true</static>" if static else ""
        x, y, z, yaw = pose
        req.sdf = (f"<sdf version='1.6'><include><uri>file://{sdf_path}</uri>"
                   f"<pose>{x} {y} {z} 0 0 {yaw}</pose>{st}</include></sdf>")
        self.request(f"/world/{self.world}/create", req)

    def set_pose(self, name, x, y, z, roll, pitch, yaw):
        req = Pose()
        req.name = name
        req.position.x, req.position.y, req.position.z = x, y, z
        w, qx, qy, qz = euler_to_quat(roll, pitch, yaw)
        req.orientation.w, req.orientation.x, req.orientation.y, req.orientation.z = w, qx, qy, qz
        self.request(f"/world/{self.world}/set_pose", req)

    def record(self, view, start, path=None):
        req = VideoRecord()
        if start:
            req.start = True
            req.format = "mp4"
            req.save_filename = path
        else:
            req.stop = True
        self.request(f"/video/{view}/record", req)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulg")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=float, default=15.0, help="pose update rate (the video frame rate is set in the camera model)")
    ap.add_argument("--start", type=float, default=None, help="log time [s] to start at (default: 2 s before take-off)")
    ap.add_argument("--end", type=float, default=None, help="log time [s] to end at (default: landing + 2 s)")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    t, x, y, z, roll, pitch, yaw = load_trajectory(args.ulg)
    z0 = z[0]
    airborne = np.where(z - z0 > 0.1)[0]
    t_start = args.start if args.start is not None else max(0.0, t[airborne[0]] - 2.0)
    t_end = args.end if args.end is not None else min(t[-1], t[airborne[-1]] + 2.0)
    log(f"trajectory {t[-1]:.1f} s, filming {t_start:.1f}-{t_end:.1f} s at {args.fps:g} fps")

    env = ff.gz_environment()
    os.environ.update(env)  # the transport node must use the same GZ_IP / partition as the server
    ff.kill_stale()
    gz = ff.start_gazebo(env, args.out, None)
    try:
        rp = Replay(ff.WORLD_NAME)
        # the first statistics sample shows that the node has discovered the server
        ff.wait_for(lambda: rp.clock.valid, 20, "world statistics", period=0.2)
        i0 = int(np.searchsorted(t, t_start))
        rp.spawn(ff.MODEL_NAME, ff.MODEL_SDF, (x[i0], y[i0], z[i0], yaw[i0]), static=True)
        rp.spawn(ff.CAMERAS_NAME, ff.CAMERAS_SDF, (0, 0, 0, 0))
        ff.wait_for(lambda: f"/video/{ff.VIDEO_VIEWS[-1]}/image" in ff.gz_topics(env), 30, "camera topics")
        time.sleep(1.0)
        for view in ff.VIDEO_VIEWS:
            rp.record(view, True, os.path.join(args.out, f"{view}.mp4"))
        sim0 = rp.clock.now()
        t_wall = time.time()
        next_report = 0.0
        while True:
            tk = t_start + (rp.clock.now() - sim0)
            if tk >= t_end:
                break
            i = min(int(np.searchsorted(t, tk)), len(t) - 1)
            rp.set_pose(ff.MODEL_NAME, x[i], y[i], z[i], roll[i], pitch[i], yaw[i])
            if tk >= next_report:
                log(f"  log t={tk:5.1f}s  alt={z[i] - z0:.2f} m  real-time factor {rp.clock.rtf:.2f}  "
                    f"({time.time() - t_wall:.0f} s elapsed)")
                next_report = tk + 5.0
            time.sleep(1.0 / (2.0 * args.fps))
        for view in ff.VIDEO_VIEWS:
            rp.record(view, False)
        time.sleep(3.0)
    finally:
        ff.stop_gazebo(gz)
    ff.combine_video(args.out)


if __name__ == "__main__":
    main()
