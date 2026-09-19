# Results

Four flights of the route in `fly_furniture.py` (indoor house world,
x500 with optical flow and a downward ToF-like range finder, `--alt 1.5`,
fast passes at 3 m/s), one per combination of `EKF2_RNG_OBST` and GPS.
Each directory holds `altitude.png`, `metrics.json` and `run_info.json`
produced by `analyze.py` / `fly_furniture.py`; the flight logs (20-30 MB
each) are not committed (`results/**/*.ulg` is ignored).

The commanded altitude is what the position controller holds: 1.33 m above
the resting position in the EKF frame (1.46 m in `obst0_nogps`; the
estimate creeps up 0.15-0.3 m between arming and take-off, so the true
altitude over the floor is ~1.15 m). "true" is the Gazebo ground truth, "est" the EKF estimate; the
cruise segment starts when the estimate has settled at the setpoint and
ends at the landing command. `dev` is the deviation from the commanded
altitude.

| run | `EKF2_RNG_OBST` | GPS | true alt min / max [m] | true dev RMS / max [m] | est - true RMS / max [m] | rng rejected | dist_bottom / z resets |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `obst0_nogps` (terrain following) | 0 | no | 1.01 / 2.08 | 0.29 / 0.63 | 0.29 / 0.68 | 10.1 % | 1 / 0 |
| `obst0_gps` (terrain following) | 0 | yes | 1.02 / 2.81 | 0.35 / 1.48 | 0.32 / 1.35 | 15.5 % | 0 / 0 |
| `obst1_nogps` (hold altitude) | 1 | no | 0.44 / 1.44 | 0.21 / 0.88 | 0.17 / 0.36 | 0.6 % | 20 / 12 |
| `obst1_gps` (hold altitude) | 1 | yes | 1.00 / 1.42 | 0.18 / 0.33 | 0.17 / 0.26 | 0.5 % | 14 / 11 |

## What the runs show

**Terrain following (`EKF2_RNG_OBST=0`)** does what the range finder
tells it. A fast pass (about 1 s over the dining table or the couch) is
rejected by the innovation gate, so the altitude survives it, but the EKF
height above ground (`dist_bottom`) does not follow the furniture at all
during the pass. As soon as an obstacle stays under the vehicle for longer
than the range height fusion timeout (the couch pass followed by the hover
above the dining table, about 6 s) the height is reset onto the surface
below: without GPS the vehicle climbs to 2.1 m and keeps a 0.3-0.4 m offset
for the rest of the flight, with GPS it climbs to 2.8 m, 1.5 m above the
commanded altitude, before the range finder wins the height back. Runs
without GPS are not repeatable in this respect: of the four `obst 0`
flights flown without GPS, two were aborted by horizontal flow drift
(one of them with a flat altitude but `dist_bottom` stuck at 1.35 m over
every obstacle) and an earlier one on the same code ran away to 4.1 m
over the couch.

**Hold altitude (`EKF2_RNG_OBST=1`)**, with or without GPS, keeps the true
altitude flat over the dining table (fast), the coffee table (slow), the
couch (fast) and the 4 s hover above the dining table: the true altitude
stays within +-0.05 m of its cruise value across all of them, the range
finder steps (0.55-0.9 m of measured distance) are attributed to the surface
below after two agreeing samples and `dist_bottom` follows the furniture.
The range innovation rejection rate is 0.5-0.6 % (only the samples between a
step and its confirmation) and every step produces one `dist_bottom` reset
and, when the vehicle leaves the obstacle, one small `z` re-anchoring reset.

The remaining deviations in the hold-altitude runs are not caused by the
range finder:

* the 0.15 m step platform (20-27 s) is below the step threshold
  (`EKF2_RNG_GATE` x sqrt(2) x the range noise) and is followed like terrain
  in both modes: +0.15 m on the platform, a 0.1-0.3 m undershoot when leaving
  it. This is the intended behaviour: a change smaller than the gate is
  indistinguishable from a height error;
* `obst1_nogps`: the 0.88 m max deviation (true altitude 0.44 m at ~75 s)
  is a physical collision with a dining chair after ~0.8 m of optical-flow
  position drift on the way back to the spawn point (accelerometer goes to
  free fall, then the impact). The EKF tracked the true altitude through it
  (est - true stays below 0.36 m); the collision is a horizontal navigation
  problem of the flow-only setup, not a height one. With GPS the same route
  was flown with < 0.1 m of position error and no contact;
* the ~0.15 m constant offset between the estimate and the truth is the
  height creep between arming and take-off mentioned above, and is the same
  in all four runs.

## Reproducing

```sh
python3 test/sitl_rng_obstacle/fly_furniture.py --obst 0 --out /tmp/obst0_nogps
python3 test/sitl_rng_obstacle/fly_furniture.py --obst 0 --gps --out /tmp/obst0_gps
python3 test/sitl_rng_obstacle/fly_furniture.py --obst 1 --out /tmp/obst1_nogps
python3 test/sitl_rng_obstacle/fly_furniture.py --obst 1 --gps --out /tmp/obst1_gps
for d in /tmp/obst*; do python3 test/sitl_rng_obstacle/analyze.py $d/*.ulg --out $d; done
```

Runs without GPS are not deterministic (see the flow drift notes in the
parent README); a run that aborts with "estimate ... from ground truth" or
"... from setpoint" drifted too far horizontally and should be repeated.
