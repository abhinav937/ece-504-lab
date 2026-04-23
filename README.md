# Wolfpack Position Loop

## Overview
The position loop sits as the outermost ring in a three-loop cascade:
```
Position Loop (P + VFF)
  └─> Speed Loop (PI + Accel FF)
        └─> Current Loop (PI per d/q axis)
              └─> SVPWM / Inverter
```
The position loop produces a speed reference. The speed loop produces a torque command. The current loop tracks the torque command by regulating d/q currents. This structure is standard in industrial servo drives.

## Position Modes
There are three position commands. They are mutually exclusive — calling any one of them switches the loop to that mode.

### Mode 1 — Single-revolution wrapped (`set_theta_m_ref`)
```
wolfpack set_theta_m_ref <rad>
```
- Reference is wrapped to **[0, 2π)** before being applied.
- Position error is wrapped to **±π**, so the motor always takes the **shortest angular path** to the target (never more than half a revolution of travel).
- Feedback is `LOG_theta_m` (the raw encoder angle, also wrapped to [0, 2π)).
- Use this for simple within-one-revolution servo positioning.

**Example:**
```
wolfpack set_theta_m_ref 0          # go to encoder zero
wolfpack set_theta_m_ref 1.5708     # go to 90° (CW from zero)
wolfpack set_theta_m_ref 4.7124     # go to 270° — motor takes shortest path
```

> **Note:** `set_theta_m_ref 7.0` and `set_theta_m_ref 0.717` command the same physical position because 7.0 wraps to `7.0 − 2π ≈ 0.717` rad.

### Mode 2 — Multi-turn absolute (`set_theta_m_ref_abs`)
```
wolfpack set_theta_m_ref_abs <rad>
```
- Reference is the **true absolute angle from encoder power-on zero**, with no wrapping.
- Feedback is `LOG_theta_m_accum` — the unwrapped accumulated angle.
- **No shortest-path routing.** The motor goes exactly to the commanded absolute angle.
- Use this when direction and absolute position matter (robot joints, lead screws, etc.).

**Example:**
```
wolfpack set_theta_m_ref_abs 0          # go to encoder power-on zero
wolfpack set_theta_m_ref_abs 31.416     # go to exactly 5 full turns CW (10π rad)
wolfpack set_theta_m_ref_abs -3.1416    # go to exactly half a turn CCW
```

### Mode 3 — Multi-turn relative (`set_theta_m_ref_rel`)
```
wolfpack set_theta_m_ref_rel <delta_rad>
```
- **Adds** `delta_rad` to the current position reference.
- On the first call (or after switching from another mode), the reference is anchored to the current shaft position (`LOG_theta_m_accum`).
- Subsequent calls stack — two calls of `5π` reach the same endpoint as one call of `10π`.

**Example:**
```
wolfpack set_theta_m_ref_rel 3.1416     # move +π rad CW
wolfpack set_theta_m_ref_rel 3.1416     # move another +π rad CW — now at 2π total
```

## How the Position Loop Works
### Proportional + Velocity Feedforward
```
error = theta_ref − theta_feedback
w_ref_prop = Kpp × error
w_vff = Kvff × (theta_ref[k] − theta_ref[k−1]) / Ts
w_ref = w_ref_prop + w_vff
w_ref = clamp(w_ref, ±w_ref_max)
```

The **proportional term** reacts to position error.  
The **velocity feedforward** differentiates the position reference each ISR cycle using a backward difference `(θ*[k] − θ*[k−1]) / T` and adds it directly to the speed reference. This anticipates how fast the target is moving and greatly reduces tracking lag on ramps and trajectories. This exactly matches the `(1 − z⁻¹)/T` block in the Industry Standard Digital Motion Controller diagram.

### Speed Clamp
The output of the position loop is clamped to `±pos_w_m_ref_max` (default 40 rad/s). This limits how aggressively the motor chases a position, preventing the speed loop from being over-driven.

## How the Speed Loop Works
### PI + CFF Torque Feedforward
```
w_error = w_ref − w_m_filtered
T_prop = Kpv × w_error
T_inte += Kiv × w_error × Ts (clamped ±T_E_MAX, anti-windup)
T_ff = speed_vff_gain × J × (w_ref[k] − w_ref[k−1]) × TASK_WOLFPACK_UPDATES_PER_SEC
T_cmd = T_prop + T_inte + T_ff
```

`T_ff = J × dω*/dt` is exactly the torque needed to accelerate the rotor inertia at the commanded rate. It is supplied before any error accumulates. The PI integrator then handles only friction, load disturbances, and model error. Set `speed_vff_gain = 0` to disable CFF and observe `LOG_T_e_cmd_inte` wind up during moves.

### Anti-windup
The speed integrator is clamped to `±T_E_MAX = 1.5 × P × λ_pm × I_lim ≈ ±0.46 N·m`. This is the maximum torque the current limiter will physically allow.

## Tuning Parameters

### Position loop gains
| Command                    | Default | Description |
|---------------------------|---------|-----------|
| `set_pos_kp <val>`        | 12.0    | Position proportional gain [rad/s per rad] |
| `set_pos_ki <val>`        | 20.0    | Position integral gain [rad/s per rad·s] |
| `set_pos_w_m_ref_max <val>` | 40.0  | Speed command clamp [rad/s] |
| `set_pos_vff_gain <val>`  | 1.0     | Velocity feedforward gain. Computes `w_vff = Kvff × (θ*[k] − θ*[k−1]) × Fs`. 1.0 = full, 0.0 = off |

### Speed loop gains
| Command                    | Default          | Description |
|---------------------------|------------------|-----------|
| `set_speed_kp <val>`      | `J × ω_gcf`      | Speed PI proportional gain [N·m·s/rad] |
| `set_speed_ki <val>`      | `B × ω_gcf`      | Speed PI integral gain [N·m/rad] |
| `set_speed_vff_gain <val>`| 1.0              | CFF torque feedforward gain. 1.0 = full inertia compensation |

### Tuning procedure
1. Start with velocity feedforward **disabled**: `set_pos_vff_gain 0`
2. Tune `set_pos_kp`, `set_pos_ki`, and speed loop gains until step response is damped with zero steady-state error
3. Re-enable velocity feedforward: `set_pos_vff_gain 1.0`
4. If overshoot appears, reduce `set_pos_vff_gain` below 1.0

## Logged Variables
| Variable                  | Description |
|---------------------------|-------------|
| `LOG_theta_m`             | Rotor angle, wrapped to [0, 2π) [rad] |
| `LOG_theta_m_accum`       | Rotor angle, unwrapped multi-turn [rad] |
| `LOG_theta_m_ref`         | Active position reference [rad] |
| `LOG_theta_m_error`       | Position error: ref − feedback [rad] |
| `LOG_w_m_ref`             | Speed reference output of position loop [rad/s] |
| `LOG_w_m_vff`             | Velocity feedforward contribution [rad/s] |
| `LOG_T_e_cmd_prop`        | Speed PI proportional torque [N·m] |
| `LOG_T_e_cmd_inte`        | Speed PI integral torque [N·m] |
| `LOG_T_e_cmd`             | Total torque command [N·m] |
| `LOG_scurve_progress`     | CFF trajectory progress 0.0 → 1.0 |
| `LOG_scurve_t_elapsed`    | CFF trajectory elapsed time [s] |
| `LOG_scurve_t_total`      | CFF trajectory total duration [s] |
| `LOG_T_e_cmd_ff`          | CFF torque feedforward contribution [N·m] |

## CFF Velocity Trajectory (Elevator Mode)

The CFF trajectory generates a **jerk-limited velocity profile** (raised-cosine) and adds **torque feedforward** (`T_ff = J × dω*/dt`) so the PI integrator does not need to wind up during acceleration.

### Numerical example: Floor 1 → Floor 3 (30 rotations)

**Command:**
```
wolfpack scurve_en 1
wolfpack scurve_goto 30
```

#### Step 1 — Trajectory planning
```
target_theta = 188.5 rad
delta = 188.5 rad
t_accel = 7.3 s
t_const = 2 × (188.5 − 131.4) / 18 = 6.3 s
t_total = 20.9 s
Check: 0.5 × 18 × 20.9 ≈ 188.5 rad ✓
```

#### Step 2 — Velocity profile
The velocity follows a smooth raised-cosine from 0 → `v_max` → 0.

#### Step 3 — CFF torque feedforward
`T_ff` pre-supplies the exact inertia torque needed. `LOG_T_e_cmd_inte` stays near zero during the move.

#### Step 4 — Position handoff
When `|target − LOG_theta_m_accum| < 0.5 rad`, the trajectory hands off to the position loop for final precision stop.

## Commands
```
wolfpack scurve_en <0|1>
wolfpack scurve_set_v_max <rad/s>
wolfpack scurve_set_a_max <rad/s²>
wolfpack scurve_set_j_max <rad/s³>
wolfpack scurve_goto <rotations>
wolfpack scurve_goto_vel <rotations>
wolfpack scurve_stop
wolfpack set_speed_vff_gain <val>
```

When `scurve_en 1`, `set_theta_m_ref_abs` and `set_theta_m_ref_rel` automatically route through the CFF trajectory.

## Startup Sequence
```
wolfpack init
wolfpack sm_calibrate
wolfpack sm_run
wolfpack set_theta_m_ref_abs 0
wolfpack set_theta_m_ref_abs 1.5708
wolfpack set_theta_m_ref_rel 3.1416
wolfpack sm_idle
```

