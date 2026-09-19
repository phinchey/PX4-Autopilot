#!/usr/bin/env python3
"""
Analyze a ulog recorded by fly_furniture.py: true vs estimated altitude while
flying over furniture with the range finder as EKF2 height reference.

    analyze.py <file.ulg> --out <dir>

Writes <out>/altitude.png (three stacked, time-aligned panels) and
<out>/metrics.json, and prints the metrics table.

Altitudes are "height gained above the resting position on the floor at the
spawn point": ground truth is vehicle_local_position_groundtruth (-z minus
its value at rest) and the estimate is vehicle_local_position (-z minus its
value at rest, taken during the first second after the range finder height
became valid).  The commanded altitude is vehicle_local_position_setpoint z
in the same frame (--alt from the flight script otherwise).  A difference
between "true" and "commanded" therefore includes any EKF height offset that
built up before takeoff.

Metrics are computed on the cruise segment: from the first time the
estimated altitude has been within 0.15 m of the commanded altitude for 1 s,
until the commander reports "Landing" (or the end of the log).
"""

import argparse
import json
import os
import sys

import numpy as np
from pyulog import ULog

# Plot styling: one consistent palette, readable in light and dark viewers.
COL_TRUE = "#1b6f9e"       # blue
COL_EST = "#d9741a"        # orange
COL_SP = "#5c5c5c"         # grey
COL_RNG = "#2a9d5c"        # green
COL_DB = "#8d4fb5"         # purple
COL_REJ = "#c8322b"        # red


def dataset(ulog, name, instance=0):
    """Fields of a logged topic, restricted to samples stamped after the log
    start (a topic can carry one sample from before the logger started, and
    its unsigned timestamp would wrap around when the start is subtracted)."""
    try:
        data = ulog.get_dataset(name, instance).data
    except (KeyError, IndexError, ValueError):
        return None
    keep = data["timestamp"] >= ulog.start_timestamp
    return {k: v[keep] for k, v in data.items()}


def as_time(data, t0):
    return (data["timestamp"].astype(np.int64) - int(t0)) * 1e-6


def rest_reference(t, y, t_from, duration=1.0):
    """Mean of y over [t_from, t_from + duration]."""
    m = (t >= t_from) & (t <= t_from + duration)
    if m.sum() < 2:
        m = t <= t[0] + duration
    return float(np.mean(y[m]))


def interp(t_query, t_src, y_src):
    return np.interp(t_query, t_src, y_src)


def arm_time(ulog, t0):
    """Time of the first armed sample (seconds from log start), or 0."""
    d = dataset(ulog, "vehicle_status")
    if d is not None:
        armed = d["arming_state"] == 2
        if armed.any():
            return float((d["timestamp"][armed][0] - t0) * 1e-6)
    return 0.0


def landing_time(ulog, t0, t_after):
    """Time of the commander's 'Landing' message after t_after, or None."""
    for m in ulog.logged_messages:
        t = (int(m.timestamp) - int(t0)) * 1e-6
        if t > t_after and "Landing" in m.message and "detected" not in m.message:
            return float(t)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulg")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--alt", type=float, default=None,
                    help="commanded altitude [m] if vehicle_local_position_setpoint is not logged")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    ulog = ULog(args.ulg)
    t0 = ulog.start_timestamp

    lp = dataset(ulog, "vehicle_local_position")
    gt = dataset(ulog, "vehicle_local_position_groundtruth")
    if lp is None or gt is None:
        sys.exit("vehicle_local_position and vehicle_local_position_groundtruth are required")
    ds = dataset(ulog, "distance_sensor")
    flags = dataset(ulog, "estimator_status_flags")
    rng = dataset(ulog, "estimator_aid_src_rng_hgt")
    sp = dataset(ulog, "vehicle_local_position_setpoint")
    t_armed = arm_time(ulog, t0)

    # ---- altitudes above the resting position
    t_lp, t_gt = as_time(lp, t0), as_time(gt, t0)
    valid = np.where(lp["dist_bottom_valid"].astype(bool))[0] if "dist_bottom_valid" in lp else np.array([0])
    t_rest = float(t_lp[valid[0]]) if len(valid) else float(t_lp[0])
    z_rest_est = rest_reference(t_lp, lp["z"], t_rest)
    z_rest_true = rest_reference(t_gt, gt["z"], t_rest)
    alt_true = -(gt["z"] - z_rest_true)
    alt_est = -(lp["z"] - z_rest_est)
    est_floor_offset = -z_rest_est  # the EKF origin sits at the range finder height above the floor

    if sp is not None and np.isfinite(sp["z"]).any():
        t_sp = as_time(sp, t0)
        alt_sp = -sp["z"] - est_floor_offset
        alt_sp_lp = interp(t_lp, t_sp, np.where(np.isfinite(alt_sp), alt_sp, np.nan))
    else:
        t_sp = None
        alt_sp_lp = np.full_like(t_lp, args.alt if args.alt else np.nan)
    if args.alt is not None:
        alt_cmd = args.alt
    else:
        finite = alt_sp_lp[np.isfinite(alt_sp_lp)]
        alt_cmd = float(np.median(finite[finite > 0.5])) if (finite > 0.5).any() else float("nan")

    # ---- cruise segment: estimate settled at the commanded altitude ... landing command
    true_on_lp = interp(t_lp, t_gt, alt_true)
    settled = (t_lp > t_armed) & (np.abs(alt_est - alt_cmd) < 0.15)
    t_start = None
    for i in np.where(settled)[0]:
        j = np.searchsorted(t_lp, t_lp[i] + 1.0)
        if settled[i:j].all():
            t_start = float(t_lp[i])
            break
    if t_start is None:
        sys.exit("the estimated altitude never settled at the commanded altitude")
    t_end = landing_time(ulog, t0, t_start) or float(t_lp[-1])
    seg = (t_lp >= t_start) & (t_lp <= t_end)

    # ---- metrics
    dev_true = true_on_lp[seg] - alt_cmd
    err_est = alt_est[seg] - true_on_lp[seg]
    metrics = {
        "ulog": os.path.basename(args.ulg),
        "commanded_alt_m": alt_cmd,
        "segment_start_s": t_start,
        "segment_end_s": t_end,
        "true_alt_max_dev_m": float(np.max(np.abs(dev_true))),
        "true_alt_rms_dev_m": float(np.sqrt(np.mean(dev_true ** 2))),
        "true_alt_min_m": float(np.min(true_on_lp[seg])),
        "true_alt_max_m": float(np.max(true_on_lp[seg])),
        "est_minus_true_rms_m": float(np.sqrt(np.mean(err_est ** 2))),
        "est_minus_true_max_m": float(np.max(np.abs(err_est))),
    }
    if "dist_bottom_reset_counter" in lp:
        c = lp["dist_bottom_reset_counter"][seg]
        metrics["dist_bottom_resets"] = int(np.sum(np.diff(c.astype(np.int16)) != 0))
    if "z_reset_counter" in lp:
        c = lp["z_reset_counter"][seg]
        metrics["z_resets"] = int(np.sum(np.diff(c.astype(np.int16)) != 0))
    if rng is not None:
        t_rng = as_time(rng, t0)
        m = (t_rng >= t_start) & (t_rng <= t_end)
        rej = rng["innovation_rejected"][m].astype(bool)
        metrics["rng_samples"] = int(m.sum())
        metrics["rng_rejected_fraction"] = float(rej.mean()) if m.sum() else float("nan")
    if flags is not None:
        t_fl = as_time(flags, t0)
        m = (t_fl >= t_start) & (t_fl <= t_end)
        for key in ("cs_rng_hgt", "cs_rng_terrain", "cs_opt_flow", "cs_gnss_pos", "cs_gnss_vel", "cs_baro_hgt"):
            if key in flags and m.sum():
                metrics[key + "_fraction"] = float(flags[key][m].astype(bool).mean())
    for name, value in ulog.initial_parameters.items():
        if name in ("EKF2_RNG_OBST", "EKF2_HGT_REF", "EKF2_RNG_CTRL", "EKF2_GPS_CTRL",
                    "EKF2_RNG_NOISE", "EKF2_RNG_GATE", "MPC_XY_VEL_MAX"):
            metrics["param_" + name] = value

    with open(os.path.join(args.out, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)
    width = max(len(k) for k in metrics)
    print(f"{'metric':<{width}}  value")
    for k, v in metrics.items():
        print(f"{k:<{width}}  {v:.3f}" if isinstance(v, float) else f"{k:<{width}}  {v}")

    # ---- plot
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 10), constrained_layout=True)
    title = os.path.basename(args.ulg)
    obst = metrics.get("param_EKF2_RNG_OBST")
    if obst is not None:
        title += f"   EKF2_RNG_OBST={obst}"
    if metrics.get("param_EKF2_GPS_CTRL", 0):
        title += "   +GPS"
    fig.suptitle(title)

    ax = axes[0]
    ax.plot(t_gt, alt_true, color=COL_TRUE, lw=1.4, label="true altitude (ground truth)")
    ax.plot(t_lp, alt_est, color=COL_EST, lw=1.2, label="estimated altitude (EKF)")
    if t_sp is not None:
        ax.plot(t_sp, alt_sp, color=COL_SP, lw=1.0, ls="--", label="setpoint altitude")
    else:
        ax.axhline(alt_cmd, color=COL_SP, lw=1.0, ls="--", label="commanded altitude")
    ax.axvspan(t_start, t_end, color="#000000", alpha=0.05, label="cruise segment (metrics)")
    ax.set_ylabel("altitude above spawn floor [m]")
    ax.legend(loc="upper right", ncol=2, fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    if ds is not None:
        t_ds = as_time(ds, t0)
        r = ds["current_distance"].astype(float)
        r[~np.isfinite(r)] = np.nan
        ax.plot(t_ds, r, color=COL_RNG, lw=1.0, label="range finder measurement")
    ax.plot(t_lp, lp["dist_bottom"], color=COL_DB, lw=1.2, label="dist_bottom (EKF height above ground)")
    if "dist_bottom_valid" in lp:
        invalid = ~lp["dist_bottom_valid"].astype(bool)
        if invalid.any():
            ax.scatter(t_lp[invalid], lp["dist_bottom"][invalid], s=6, color=COL_REJ, label="dist_bottom invalid")
    ax.set_ylabel("distance [m]")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)

    ax = axes[2]
    if rng is not None:
        t_rng = as_time(rng, t0)
        ax.plot(t_rng, rng["innovation"], color=COL_RNG, lw=0.9, label="rng height innovation")
        rej = rng["innovation_rejected"].astype(bool)
        if rej.any():
            ax.scatter(t_rng[rej], rng["innovation"][rej], s=8, color=COL_REJ, zorder=3, label="innovation rejected")
    ax.set_ylabel("innovation [m]")
    ax.grid(True, alpha=0.3)
    ax2 = ax.twinx()
    if "dist_bottom_reset_counter" in lp:
        ax2.step(t_lp, lp["dist_bottom_reset_counter"], where="post", color=COL_DB, lw=1.0,
                 label="dist_bottom reset counter")
    if "z_reset_counter" in lp:
        ax2.step(t_lp, lp["z_reset_counter"], where="post", color=COL_TRUE, lw=1.0, ls="--",
                 label="z reset counter")
    ax2.set_ylabel("reset counter")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=9)
    ax.set_xlabel("time since log start [s]")

    png = os.path.join(args.out, "altitude.png")
    fig.savefig(png, dpi=120)
    print(f"\nwrote {png}\nwrote {os.path.join(args.out, 'metrics.json')}")


if __name__ == "__main__":
    main()
