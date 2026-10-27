# Assumptions, models and parameter ranges

All units are SI (m, s, rad, m/s, m/s²). Constants live in `server/sim/constants.js`, road geometry in
`server/sim/road.js`, traffic parameters in `server/sim/traffic.js`.

## Coordinate frame and road

- World frame: `x` along the highway in the direction of travel, `y` to the left of travel. Heading
  is measured counter-clockwise from +x; positive steering turns left. The browser draws +y upwards.
- Main carriageway: three lanes, 3.7 m wide, `x ∈ [-400, 1600]`, `y ∈ [0, 11.1]`. Lane 0 is the
  rightmost lane (`y ∈ [0, 3.7]`), lane 2 the leftmost.
- Entrance ramp: 4.0 m wide straight ramp from (-60, -52) to (160, -1.85), about 226 m long and
  angled at about 12.8°. It joins an acceleration lane (`x ∈ [150, 380]`, `y ∈ [-3.7, 0]`) that
  tapers out at x = 440.
- Exit: the deceleration lane tapers in from x = 820, runs at full width over `x ∈ [880, 1000]`, and
  feeds a 4.0 m wide exit ramp from (990, -1.85) to (1200, -50). The road ends there.
- The drivable surface is the union of seven convex polygons (`REGIONS`). Anything else, including
  the gore areas between the ramps and the carriageway, counts as off-road.
- Speed limits: 30 m/s (108 km/h) on the carriageway, acceleration, deceleration and taper areas;
  22 m/s (79 km/h) on both ramps. The limit is taken from the region containing the ego's centre.

## Time stepping

- Physics timestep `PHYS_DT = 0.02 s` (50 Hz), fixed. Simulated time after *n* steps is exactly
  `n · PHYS_DT`.
- The controller runs every 5 steps (`CONTROL_DT = 0.1 s`, 10 Hz). Its command is held (zero-order
  hold) until the next control step.
- The server runs steps in real time scaled by the playback speed (×0.25 … ×16). Wall-clock time
  only decides *how many* fixed steps to run. The browser just renders the latest state. Results
  are therefore independent of server tick rate and rendering frame rate (covered by a test).
- **Single step** in the UI advances one control period (5 physics steps), so each press produces a
  new controller command.

## Vehicle model

Kinematic bicycle model referenced at the body centre (rear axle distance `lr = wheelbase/2`):

```
β = atan(½ tan δ)
x += v̄ cos(ψ + β) dt,   y += v̄ sin(ψ + β) dt,   ψ += (v̄ / lr) sin β dt,   v̄ = (v_k + v_{k+1}) / 2
```

Actuator limits are applied identically to every command. They are the plant's physics, not a
correction applied by the safety checker:

| Limit | Ego | Car | Truck |
|---|---|---|---|
| Acceleration range (m/s²) | [-8, 3] | [-9, 2.5] | [-7, 1.2] |
| Steering angle (rad) | ±0.5 | ±0.5 | ±0.5 |
| Steering rate (rad/s) | 0.7 | 0.5 | 0.3 |
| Tyre lateral-acceleration limit (m/s²) | 8 | 4 | 3 |
| Speed (m/s) | [0, 45] | [0, 45] | [0, 30] |
| Length × width (m), wheelbase | 4.7 × 1.9, 2.85 | 4.6 × 1.9, 2.8 | 12 × 2.55, 7.5 |

- The usable steering angle shrinks with speed: `|δ| ≤ min(δmax, atan(a_lat,max · L / v²))`.
  For the ego that is about 0.025 rad at 30 m/s. The controller receives this value each step as
  `steer_max_now`.
- Vehicles do not reverse: braking stops at v = 0.
- The ego's valid command range is `accel ∈ [-8, 3]`, `steer ∈ [-0.5, 0.5]`.

## Surrounding traffic

- **Longitudinal (IDM):** `s0 = 2 m`, δ = 4, `b = 2 m/s²`, `a = 1.4 m/s²` for cars and 0.8 m/s² for
  trucks, time headway `T ∈ [1.1, 1.7] s` per vehicle. The leader is the nearest vehicle ahead in
  any lane the vehicle occupies or is moving into. The ego counts like any other vehicle.
- **Lane occupancy:** a vehicle occupies a lane when its body overlaps the lane laterally by at
  least 0.3 m. A vehicle that is changing lanes also claims its target lane, like a turn signal, so
  followers in that lane start reacting early.
- **Lane changes (MOBIL):**
  - Each vehicle evaluates every 0.5 s.
  - Politeness is in [0.1, 0.5], the change threshold is 0.2 m/s², and the new follower must not
    have to brake harder than 4 m/s². There is a 2 m minimum gap and a 4 s cooldown after each
    change.
  - Keep-right bias: +0.25 m/s².
  - Courtesy: a vehicle in lane 0 within 50 m of an ego on the acceleration lane gets +0.6 m/s²
    toward moving left, to make room for the merge.
- **Lateral control:** NPCs track the target lane centre through the same bounded bicycle model.
  The desired heading is at most 0.07 rad, which gives lane changes of roughly 3–4 s.
- **Desired speeds** per lane are `base − 3`, `base`, `base + 3` m/s for lanes 0, 1 and 2, plus a
  per-vehicle offset in [−2, +2.5] m/s. About 6 % of lane-0 vehicles are trucks.
- **Inflow:** new vehicles enter at x = −385 per lane as a Poisson process. The rate is
  `density · lane speed / 1000` vehicles per second, capped at 1. A vehicle only enters when the
  entry gap is safe. Vehicles are removed at x > 1590.
- **Observation:** the controller sees ground truth for every surrounding vehicle within 120 m
  (centre to centre), nearest first, at most 64. There is no occlusion or sensor noise.
- All randomness comes from a seeded PRNG (mulberry32). Scenario generation uses `seed`; traffic
  inflow uses a stream derived from `seed`.

## Scenarios and parameter ranges

| Scenario | Parameter | Range | Default |
|---|---|---|---|
| `merge_busy` – start on the entrance ramp; destination is the highway at x ∈ [600, 700]; time limit 60 s | `density` (veh/km/lane) | 5 – 45 | 28 |
| | `traffic_speed` (m/s, lane 1 base) | 15 – 30 | 24 |
| | `ego_speed` (m/s) | 8 – 22 | 15 |
| | `ego_ramp_pos` (m along ramp) | 20 – 120 | 50 |
| `slow_vehicle` – middle lane, a truck ahead that never changes lanes; destination x ∈ [1100, 1200]; 70 s | `slow_speed` (m/s) | 6 – 22 | 14 |
| | `slow_gap` (m) | 30 – 200 | 90 |
| | `ego_speed` (m/s) | 15 – 30 | 26 |
| | `density` | 0 – 30 | 12 |
| `cut_in` – a car from the left or right lane (chosen by the seed) cuts into the ego lane; destination x ∈ [900, 1000]; 50 s | `ego_speed` (m/s) | 15 – 30 | 25 |
| | `cut_gap` (m, gap in front of ego when the cut-in starts) | 5 – 40 | 12 |
| | `cut_time` (s) | 1 – 10 | 3 |
| | `density` | 0 – 30 | 10 |
| `sudden_brake` – leader in the ego lane brakes hard, holds, then resumes; destination x ∈ [1000, 1100]; 60 s | `ego_speed` (m/s, also the leader's) | 10 – 30 | 25 |
| | `lead_gap` (m) | 10 – 80 | 28 |
| | `brake_decel` (m/s²) | 2 – 9 | 7 |
| | `brake_time` (s) | 1 – 15 | 4 |
| | `final_speed` (m/s) | 0 – 20 | 0 |
| | `hold_time` (s) | 0 – 30 | 8 |
| | `density` | 0 – 30 | 12 |
| `exit_traffic` – start in lane 2 (leftmost) and reach the last part of the exit ramp; 75 s | `distance_to_exit` (m to x = 880) | 250 – 800 | 550 |
| | `ego_speed` (m/s) | 15 – 30 | 26 |
| | `density` | 0 – 40 | 24 |
| | `traffic_speed` (m/s) | 15 – 30 | 25 |

Seeds are integers in [0, 4 294 967 295]. The same (scenario, seed, parameters) always produces the
same starting configuration and, with a deterministic controller, the same run. **Reset** rebuilds
the state from a frozen copy of the starting configuration and restarts the controller process.

## Rejected starting configurations

`validateConfig` (`server/sim/validate.js`) rejects a configuration, and never repairs it, when any
of these hold:

- The seed is not an integer in range, a parameter is unknown, non-numeric or out of range, or the
  time limit is outside [5, 600] s.
- A vehicle has a missing or non-finite field, a speed outside [0, 40] m/s (ego: [0, 45]), or
  dimensions out of range (cars 3–6 × 1.5–2.2 m, trucks 6–20 × 2.0–2.6 m, wheelbase ≤ 0.9 · length).
- An NPC is not centred in its lane (|Δy| > 0.6 m), has |heading| > 0.1 rad, or has a duplicate ID.
  There are more than 400 vehicles.
- Any vehicle footprint is not entirely on the drivable surface.
- Two vehicles overlap, with footprints inflated by 0.5 m.
- **An unavoidable collision at the start:** for every pair sharing a lane, the bumper gap must be at
  least `2 m + 0.3 s · v_f + max(0, v_f² − v_l²) / (2 · 6 m/s²)`. That is the distance a follower
  at `v_f` needs, after a 0.3 s reaction, to match a leader at `v_l` with 6 m/s² braking. Example:
  `slow_vehicle` with `ego_speed = 30`, `slow_speed = 6`, `slow_gap = 30` is rejected even though
  each value is within its own range.
- A scripted event references an unknown vehicle, happens outside [0, time limit], or has invalid
  arguments.
- The destination polygon is not convex, has fewer than 3 or more than 16 points, has an area under
  1 m², or its centre is off-road.
- Traffic inflow or lane speed arrays are malformed or out of range.

## Safety checker

`server/sim/safety.js` only observes. It receives the raw command and the vehicle poses before and
after each step, and it writes only to its own event list. The simulation reads `terminated` to
stop the episode.

| Event | Terminal | Definition |
|---|---|---|
| `collision` | yes | The ego footprint touches another vehicle's footprint at any instant of the step. |
| `road_departure` | yes | Any point on the ego footprint boundary (sampled at ≤ 1 m spacing) leaves the drivable surface at any instant of the step. |
| `speed_violation` / `_end` | no | Speed above the local limit + 0.1 m/s. One event at onset and one at the end, with duration and maximum excess. |
| `invalid_output` | no | Finite command outside the valid range. Reported once per run of consecutive invalid commands; the actuator saturates. |
| `invalid_output` | yes | NaN, infinite, missing or non-numeric command; controller crash, protocol error, or no reply within 2 s. |
| `arrival` | yes | Ego centre inside the destination polygon. |
| `timeout` | yes | Simulated time reached the scenario's time limit. |
| `traffic_collision` | no | Information only: two surrounding vehicles touched. |

**Continuous collision checking.** Within one step the integrator moves each centre along a straight
segment while the heading changes linearly. The checker interpolates poses the same way and tests
footprints (separating-axis test) at sample points no more than 0.1 m of relative motion apart,
including both endpoints. Vehicles cannot pass through each other between samples, and first contact
is located to within 0.1 m. Road departure uses the same sweep.

If several terminal conditions occur in the same step, the earliest within the step wins. A collision
wins a tie with a departure. After those come arrival, then timeout. Nothing is recorded after the
episode has ended.

## Known simplifications

- Straight road segments only. There are no curves, no road gradient, no tyre slip beyond the
  lateral-acceleration cap, and no aerodynamic drag (zero acceleration keeps speed constant).
- Surrounding traffic never uses the ramps, acceleration lane or deceleration lane.
- Perception is perfect within the sensor radius.
- Collision response is not simulated: an ego collision ends the episode.
