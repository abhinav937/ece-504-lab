# Wolfpack Position Loop

## Overview
The position loop sits as the outermost ring in a three-loop cascade:
```
Position Loop (PI + VFF)
  └─> Speed Loop (PI + CFF Torque Feedforward)
        └─> MTPA decomposition
              └─> Current Vector Limiter (8 A)
                    └─> Current Loop (synchronous-frame PI per d/q axis)
                          └─> Inverse Park → SVPWM / Inverter
```
The position loop produces a speed reference. The speed loop produces a torque command. MTPA decomposes the torque command into d/q current references. The current loop tracks those references by regulating d/q voltages. This structure is standard in industrial servo drives.

### System constants

| Constant | Value | Units |
|----------|-------|-------|
| Control rate | 10 000 | Hz (ISR) |
| Encoder counts/rev | 20 000 | counts |
| Pole pairs | 4 | — |
| PM flux linkage (λ_pm) | 0.0383 | Wb |
| d-axis inductance (L_d) | 1.0 | mH |
| q-axis inductance (L_q) | 1.6 | mH |
| Stator resistance (R_s) | 0.55 | Ω |
| Rotor inertia (J) | 0.0042668 | kg·m² |
| Rotational damping (B) | 0.0020483 | N·m·s/rad |
| Vector current limit | 8 | A |
| OV trip threshold | 70 | V |
| OC trip threshold (per phase) | 12 | A |

---

## Startup Sequence

```
wolfpack init
wolfpack sm_calibrate              # motor at rest, DC supply off — zeros sensor offsets
wolfpack sm_run
# manually position motor shaft at floor 0 (home position)
wolfpack zero_accum                # zeros accumulator AND drives shaft to encoder zero
wolfpack scurve_en 1
wolfpack elevator_floor 1          # go to floor 1  (10 rotations)
wolfpack elevator_floor 0          # return to zero
wolfpack sm_idle
```

---

## Protection System

The firmware implements auto-latching over-voltage (OV) and per-phase over-current (OC) protection. When either trip fires the state machine enters TRIPPED (state 3), PWMs are disabled, and the fault latches until explicitly cleared.

| Fault | Condition | Log variable |
|-------|-----------|-------------|
| Over-voltage | `LOG_v_dc > 70 V` | `LOG_OV_status_dc` |
| Over-current phase A | `|LOG_i_a| > 12 A` | `LOG_OC_status_a` |
| Over-current phase B | `|LOG_i_b| > 12 A` | `LOG_OC_status_b` |
| Over-current phase C | `|LOG_i_c| > 12 A` | `LOG_OC_status_c` |
| Combined fault | any of the above | `LOG_protection_status` |

**To recover from a trip:**
1. Resolve the root cause (reduce load, check bus voltage)
2. Issue `wolfpack sm_trip_clear`
3. Re-enter RUNNING with `wolfpack sm_run`

**Max safe operating speed: ~80 rad/s.** At higher speeds, the q-axis back-EMF
(`λ_pm × p × ω_m ≈ 0.0383 × 4 × ω_m`) approaches half the bus voltage, leaving no
headroom for the current regulators. Regenerative braking from high speed also spikes
the DC bus above 70 V and triggers OV. Field weakening is not implemented. Diagnose
runaway trips with `LOG_OV_status_dc` and `LOG_OC_status_a/b/c`.

---

## Homing with `zero_accum`

`zero_accum` is the mandatory step before any S-curve or elevator motion. It:

1. Zeros `LOG_theta_m_accum` at the current shaft position
2. Drives the motor to encoder angle 0 (shortest path via wrapped position loop)
3. Sets the internal `zero_accum_done` flag that gates all S-curve commands

Any `elevator_floor` or `scurve_goto` called before `zero_accum` is silently ignored.

`sm_calibrate` clears `zero_accum_done`, requiring re-homing after each calibration.

---

## Position Modes

There are three direct position commands. They are mutually exclusive — calling any one switches the loop to that mode. When `scurve_en 1` is active, modes 2 and 3 route through the S-curve trajectory instead of stepping the reference directly.

### Mode 1 — Single-revolution wrapped (`set_theta_m_ref`)
```
wolfpack set_theta_m_ref <rad>
```
- Reference is wrapped to **[0, 2π)** before being applied.
- Position error is wrapped to **±π**, so the motor always takes the **shortest angular path** (never more than half a revolution of travel).
- Feedback is `LOG_theta_m` (raw encoder angle, also wrapped to [0, 2π)).
- Use this for simple within-one-revolution servo positioning.

**Example:**
```
wolfpack set_theta_m_ref 0          # go to encoder zero
wolfpack set_theta_m_ref 1.5708     # go to 90°
wolfpack set_theta_m_ref 4.7124     # go to 270° — motor takes shortest path
```

### Mode 2 — Multi-turn absolute (`set_theta_m_ref_abs`)
```
wolfpack set_theta_m_ref_abs <rad>
```
- Reference is the **absolute angle from the accumulator zero**, with no wrapping.
- Feedback is `LOG_theta_m_accum` — the unwrapped accumulated angle.
- No shortest-path routing. The motor goes exactly to the commanded absolute angle.
- When `scurve_en 1`, routes through the CFF S-curve trajectory.

**Example:**
```
wolfpack set_theta_m_ref_abs 0          # go to accumulator zero
wolfpack set_theta_m_ref_abs 62.832     # go to 10 full turns (10 × 2π rad)
```

### Mode 3 — Multi-turn relative (`set_theta_m_ref_rel`)
```
wolfpack set_theta_m_ref_rel <delta_rad>
```
- **Adds** `delta_rad` to the current position reference.
- On the first call (or after switching from another mode), the reference is anchored to `LOG_theta_m_accum`.
- Subsequent calls stack — two calls of `5π` reach the same endpoint as one call of `10π`.
- When `scurve_en 1`, routes through the CFF S-curve trajectory.

---

## `zero_accum`

```
wolfpack zero_accum
```

Zeros the accumulator at the current shaft position and drives the motor to `theta_m = 0` (encoder index, shortest path via the wrapped position loop). Run this after Z-pulse is LOCKED to establish a clean positional reference.

After `zero_accum`:
- `LOG_theta_m_accum = 0.0`
- Position integrator and error states are cleared
- Motor holds at encoder zero

---

## How the Position Loop Works

```
error        = theta_ref − theta_feedback
w_ref_prop   = Kpp × error
w_ref_inte  += Ki × error × Ts         (anti-windup: only integrate when not saturated)
w_vff        = Kvff × (theta_ref[k] − theta_ref[k−1]) × Fs
w_ref        = clamp(w_ref_prop + w_ref_inte + w_vff, ±w_ref_max)
```

`theta_feedback` is `LOG_theta_m_accum` when `pos_use_accum = 1` (multi-turn modes and after S-curve handoff), or `LOG_theta_m` when `pos_use_accum = 0` (single-rev mode and during `zero_accum`).

In single-rev mode, position error is additionally wrapped to **±π** for shortest-path tracking.

The **velocity feedforward** differentiates the position reference each ISR cycle `(θ*[k] − θ*[k−1]) × Fs` and adds it directly to the speed reference. This anticipates trajectory motion and reduces tracking lag.

---

## How the Speed Loop Works

```
w_error    = w_ref − w_m_filtered
T_prop     = Kpv × w_error
T_inte    += Kiv × w_error × Ts        (clamped ±T_E_MAX, anti-windup)
T_ff       = speed_vff_gain × (J × dω*/dt + B × ω*)
T_cmd      = T_prop + T_inte + T_ff    (clamped ±T_E_MAX)
```

The CFF torque feedforward uses the full ME547 NSD formula:
```
T_ff = speed_vff_gain × (J_ESTIMATE × (w_ref[k] − w_ref[k−1]) × Fs + B_ESTIMATE × w_ref[k])
```
- `J_ESTIMATE = 0.0042668 kg·m²` — rotor inertia (from system ID)
- `B_ESTIMATE = 0.0020483 N·m·s/rad` — rotational damping (from system ID)

`T_ff` is active **only during an active S-curve trajectory** (`scurve_active = 1`). It is zeroed outside the trajectory to prevent torque impulses on direct position steps.

Set `speed_vff_gain = 0` to disable CFF and observe `LOG_T_e_cmd_inte` wind up during moves.

---

## Current Loop & MTPA

### MTPA decomposition

The torque command is first converted to a stator current magnitude, then decomposed into d/q references using the Interior PM MTPA formula (active when `mtpa_en = 1`):

```
i_s = T_cmd / (1.5 × p × λ_pm)
ΔL  = L_q − L_d
i_d = (λ_pm − √(λ_pm² + 8·ΔL²·i_s²)) / (4·ΔL)
i_q = sign(i_s) × √(i_s² − i_d²)
```

When `mtpa_en = 0`: `i_q = i_s`, `i_d = i_d_ref_manual` (manual d-axis injection).

The resulting current vector is then **magnitude-clamped to 8 A** (preserving angle). Observe the effect of limiting via `LOG_i_d_ref_limited` and `LOG_i_q_ref_limited` vs. `LOG_i_d_ref` and `LOG_i_q_ref`.

### Synchronous-frame PI current regulators

Two independent PI regulators run in the rotating (d/q) frame at 10 kHz:

```
v_d = Kpd × e_d + Kid × ∫e_d                    (no BEMF term on d-axis)
v_q = λ_pm × ω_e + Kpq × e_q + Kiq × ∫e_q      (back-EMF compensation on q-axis)
```

The q-axis BEMF term (`λ_pm × ω_e`) is a static model-based decoupling that removes the dominant speed-dependent disturbance, reducing the burden on the PI integrator at speed. There is no state-feedback cross-coupling decoupling between axes.

Voltage commands are inverse-Park transformed to abc, then SVPWM common-mode is applied before writing duty cycles to the PWM peripheral.

---

## CFF S-Curve Trajectory (Elevator Mode)

The trajectory generates a **3-phase trapezoidal velocity profile** (ramp-up → cruise → ramp-down), adds torque feedforward so the PI integrator does not need to wind up during acceleration, then hands off to the position loop for a precise stop.

### Profile phases

| Phase | `LOG_scurve_phase` | Entry condition |
|-------|--------------------|-----------------|
| Accel / Cruise | 0 | Default until braking threshold crossed |
| Decel | 1 | `remaining ≤ braking_dist + 0.05` — **latched, cannot return to phase 0** |

The decel entry is a one-way latch (`scurve_decel_latched`). Once braking begins the profile only ever decelerates — it never re-enters phase 0. Without the latch, `v_cur` and `braking_dist` both shrink each step and can push the condition back above the threshold, causing rapid 0↔1 toggling that corrupts the velocity profile.

### Trajectory initialization (`start_scurve_cff`)

Called by `scurve_goto`, `scurve_goto_vel`, `elevator_floor`, and (when `scurve_en=1`) `set_theta_m_ref_abs/_rel`.

```
// Guards (silent return on failure)
if (!zero_accum_done)               → ignored, zero_accum required first
if (|target − accum| < 0.01 rad)    → ignored, already at target

// Setup
scurve_target_theta  = target_rotations × 2π
scurve_initial_delta = |target_theta − LOG_theta_m_accum|
scurve_direction     = +1.0  if  target ≥ current,  else  −1.0
scurve_v_cur         = 0.0              // always starts from rest
scurve_decel_latched = 0
LOG_theta_m_ref      = LOG_theta_m_accum   // seed at current position
scurve_active        = 1
en_position_loop     = 0                // open-loop during CFF trajectory
```

### Per-ISR update (`update_scurve_cff`, runs every 100 µs when `scurve_active`)

```
remaining    = |scurve_target_theta − LOG_theta_m_accum|
braking_dist = scurve_v_cur² / (2 × a_max)

// One-way decel latch
if (!scurve_decel_latched  &&  remaining ≤ braking_dist + 0.05):
    scurve_decel_latched = 1

// Velocity ramp
if (scurve_decel_latched):
    scurve_v_cur -= a_max × Ts          // decelerate, clamped ≥ 0
else:
    scurve_v_cur += a_max × Ts          // accelerate, clamped ≤ v_max

// Speed and position references (both updated every ISR)
LOG_w_m_ref      = scurve_direction × scurve_v_cur
LOG_theta_m_ref += LOG_w_m_ref × Ts    // integrate position along trajectory

// Distance-based progress: 0.0 at start → 1.0 at target
LOG_scurve_progress = 1.0 − (remaining / scurve_initial_delta)

// Handoff: velocity near zero → close position loop
if (scurve_decel_latched  &&  scurve_v_cur ≤ 0.3 rad/s):
    scurve_active        = 0
    en_position_loop     = 1,  pos_use_accum = 1
    LOG_theta_m_ref      = scurve_target_theta   // small snap (tracking error only)
    speed + position integrators cleared
    scurve_decel_latched = 0                     // ready for next move
```

Short moves: if `braking_dist ≥ remaining` from the very first ISR, the decel latch fires immediately — the motor decelerates without ever accelerating.

`scurve_stop` aborts in place: zeroes velocity, clears `scurve_active`, and calls `set_theta_m_ref_abs(LOG_theta_m_accum)` to hold the current position.

### Worked example: `elevator_floor 1` (10 rotations, 62.83 rad)

Default parameters: `v_max = 18.0 rad/s`, `a_max = 2.5 rad/s²`, `Ts = 100 µs`

Each ISR adds `Δv = a_max × Ts = 0.00025 rad/s`.

The total distance is 62.83 rad. To accelerate all the way to `v_max` and then brake back to zero would need `v_max² / a_max = 18² / 2.5 = 129.6 rad` — more than twice the floor spacing. So **v_max is never reached**; the profile is a pure triangle (accel, then immediately decel at the midpoint).

| t (s) | v_cur (rad/s) | dist covered (rad) | remaining (rad) | braking_dist (rad) | phase |
|-------|--------------|-------------------|-----------------|-------------------|-------|
| 0.00 | 0.000 | 0.000 | 62.832 | 0.000 | accel |
| 1.00 | 2.500 | 1.250 | 61.582 | 1.250 | accel |
| 2.00 | 5.000 | 5.000 | 57.832 | 5.000 | accel |
| 3.00 | 7.500 | 11.250 | 51.582 | 11.250 | accel |
| 4.00 | 10.000 | 20.000 | 42.832 | 20.000 | accel |
| 5.00 | 12.500 | 31.250 | 31.582 | 31.250 | accel |
| **5.01** | **12.528** | **31.391** | **31.441** | **31.391** | **← decel latch fires** |
| 6.01 | 10.028 | 42.669 | 20.163 | 20.113 | decel |
| 7.01 | 7.528 | 51.447 | 11.385 | 11.335 | decel |
| 8.01 | 5.028 | 57.725 | 5.106 | 5.056 | decel |
| 9.01 | 2.528 | 61.504 | 1.328 | 1.278 | decel |
| **9.90** | **0.300** | **62.764** | **0.068** | **0.018** | **← handoff to position loop** |

**Decel latch at t = 5.01 s:** `remaining (31.441) ≤ braking_dist + 0.05 (31.441)` — the condition is met. From this ISR forward `v_cur` only ever decrements. Without the latch, `v_cur` and `braking_dist` would shrink together each step and the condition would toggle on and off every few ISRs.

**Handoff at t = 9.90 s:** `v_cur` hits 0.3 rad/s with 0.068 rad still to go. The trajectory ends, `LOG_theta_m_ref` snaps to the exact target (62.832 rad), and the position loop closes the remaining 0.068 rad. Total trajectory time: **9.9 s**.

### Position reference integration (key detail)

`LOG_theta_m_ref` is integrated each ISR alongside the velocity command. This keeps the position reference in sync with the trajectory so that at handoff the final snap to `scurve_target_theta` is a small correction (speed-tracking error only, typically < 0.1 rad) rather than the full remaining distance the motor may have undershot.

### Handoff to position loop

When the decel latch is active and `scurve_v_cur ≤ 0.3 rad/s`, the trajectory transfers control to the position loop:
- `en_position_loop = 1`, `pos_use_accum = 1`
- `LOG_theta_m_ref` snapped to `scurve_target_theta` (small correction, not a large jump)
- Speed and position integrators cleared for a bumpless transfer
- `scurve_decel_latched` reset to 0 for the next move

### Enable flow

```
scurve_en = 0  →  set_theta_m_ref_abs / _rel step the reference directly
scurve_en = 1  →  set_theta_m_ref_abs / _rel route through start_scurve_cff
```

`scurve_goto` and `scurve_goto_vel` always call `start_scurve_cff` directly (ignoring `scurve_en`). Arguments are in **rotations**, not radians.

---

## Elevator Floor Commands

```
wolfpack elevator_floor <N>              # go to floor N  (N × floor_spacing rotations)
wolfpack elevator_set_spacing <rot>      # set rotations per floor (default 10.0)
```

`elevator_floor 0` returns to the `zero_accum` reference. `elevator_floor N` targets `N × floor_spacing` rotations via the S-curve trajectory.

---

## Tuning Parameters

### Position loop
| Command | Default | Description |
|---------|---------|-------------|
| `set_pos_kp <val>` | 12.0 | Proportional gain [rad/s per rad] |
| `set_pos_ki <val>` | 20.0 | Integral gain [rad/s per rad·s] |
| `set_pos_w_m_ref_max <val>` | 40.0 | Speed command clamp [rad/s] |
| `set_pos_vff_gain <val>` | 1.0 | Velocity feedforward gain (1.0 = full, 0 = off) |

### Speed loop
| Command | Default | Description |
|---------|---------|-------------|
| `set_speed_kp <val>` | `J × ω_gcf` | PI proportional gain [N·m·s/rad] |
| `set_speed_ki <val>` | `B × ω_gcf` | PI integral gain [N·m/rad] |
| `set_speed_vff_gain <val>` | 1.0 | CFF torque feedforward gain (1.0 = full, 0 = off) |

### Current loop
| Command | Default | Description |
|---------|---------|-------------|
| `set_ireg_kpd <val>` | — | d-axis PI proportional gain |
| `set_ireg_kid <val>` | — | d-axis PI integral gain |
| `set_ireg_kpq <val>` | — | q-axis PI proportional gain |
| `set_ireg_kiq <val>` | — | q-axis PI integral gain |
| `set_mtpa_en <0\|1>` | 1 | MTPA enable (0 = manual i_d via set_i_d_ref_manual) |

### S-curve trajectory
| Command | Default | Description |
|---------|---------|-------------|
| `scurve_set_v_max <rad/s>` | 18.0 | Peak velocity |
| `scurve_set_a_max <rad/s²>` | 2.5 | Max acceleration (ramp rate) |
| `scurve_set_j_max <rad/s³>` | 25.0 | Reserved for future jerk-limited (NSD 7-phase) profile |

### Tuning procedure
1. Disable velocity feedforward: `set_pos_vff_gain 0`
2. Tune `set_pos_kp` and `set_pos_ki` until step response is damped with zero steady-state error
3. Re-enable: `set_pos_vff_gain 1.0`
4. If overshoot appears, reduce `set_pos_vff_gain` below 1.0

---

## Logged Variables

### Position & trajectory
| Variable | Description |
|----------|-------------|
| `LOG_theta_m` | Rotor angle, wrapped to [0, 2π) [rad] |
| `LOG_theta_m_accum` | Rotor angle, unwrapped multi-turn [rad] |
| `LOG_theta_m_ref` | Active position reference [rad] |
| `LOG_theta_m_error` | Position error: ref − feedback [rad] |
| `LOG_theta_m_fb` | Active feedback signal used by regulator [rad] |
| `LOG_pos_use_accum` | 0 = wrapped feedback, 1 = accumulator feedback |
| `LOG_w_m_ref` | Speed reference from position loop [rad/s] |
| `LOG_w_m_vff` | Velocity feedforward contribution [rad/s] |
| `LOG_scurve_progress` | Trajectory progress 0.0 → 1.0 (distance-based) |
| `LOG_scurve_remaining` | Remaining distance to target [rad] |
| `LOG_scurve_phase` | Trajectory phase: 0=accel/cruise, 1=decel |

### Speed & torque
| Variable | Description |
|----------|-------------|
| `LOG_w_m` | Measured rotor speed, filtered [rad/s] |
| `LOG_w_m_RPM` | Measured rotor speed [RPM] |
| `LOG_T_e_cmd_prop` | Speed PI proportional torque [N·m] |
| `LOG_T_e_cmd_inte` | Speed PI integral torque [N·m] |
| `LOG_T_e_cmd_ff` | CFF torque feedforward [N·m] |
| `LOG_T_e_cmd` | Total torque command [N·m] |

### Currents & voltages
| Variable | Description |
|----------|-------------|
| `LOG_i_a`, `LOG_i_b`, `LOG_i_c` | Phase currents [A] |
| `LOG_v_dc` | DC bus voltage [V] |
| `LOG_i_d`, `LOG_i_q` | Park-transformed d/q currents [A] |
| `LOG_i_d_ref`, `LOG_i_q_ref` | d/q current references before vector limiting [A] |
| `LOG_i_d_ref_limited`, `LOG_i_q_ref_limited` | d/q references after 8 A vector clamp [A] |
| `LOG_v_cmd_d`, `LOG_v_cmd_q` | dq voltage commands [V] |
| `LOG_duty_a`, `LOG_duty_b`, `LOG_duty_c` | PWM duty cycles [0–1] |

### Fault & diagnostics
| Variable | Description |
|----------|-------------|
| `LOG_OV_status_dc` | 1 if DC over-voltage trip is active |
| `LOG_OC_status_a`, `LOG_OC_status_b`, `LOG_OC_status_c` | 1 if per-phase OC trip active |
| `LOG_protection_status` | Combined protection fault flag |
| `LOG_wolf_state` | Current state machine value (0=CALIBRATE, 1=IDLE, 2=RUNNING, 3=TRIPPED) |
| `LOG_control_looptime` | ISR scheduling period [s] |
| `LOG_control_runtime` | ISR execution time [s] |


---

## Full Command Reference

```
wolfpack init
wolfpack deinit
wolfpack sm_get_state
wolfpack sm_idle
wolfpack sm_calibrate
wolfpack sm_run
wolfpack sm_trip_clear

wolfpack set_theta_m_ref <rad>
wolfpack set_theta_m_ref_abs <rad>
wolfpack set_theta_m_ref_rel <delta_rad>
wolfpack set_w_m_ref <rad/s>

wolfpack set_pos_kp <val>
wolfpack set_pos_ki <val>
wolfpack set_pos_w_m_ref_max <val>
wolfpack set_pos_vff_gain <val>
wolfpack set_speed_kp <val>
wolfpack set_speed_ki <val>
wolfpack set_speed_vff_gain <val>
wolfpack set_ireg_kpd <val>
wolfpack set_ireg_kid <val>
wolfpack set_ireg_kpq <val>
wolfpack set_ireg_kiq <val>
wolfpack set_mtpa_en <0|1>

wolfpack scurve_en <0|1>
wolfpack scurve_set_v_max <rad/s>
wolfpack scurve_set_a_max <rad/s²>
wolfpack scurve_set_j_max <rad/s³>
wolfpack scurve_goto <rotations>
wolfpack scurve_goto_vel <rotations>
wolfpack scurve_stop

wolfpack zero_accum
wolfpack elevator_floor <N>
wolfpack elevator_set_spacing <rotations_per_floor>

wolfpack set_i_q_ref_manual <A>
wolfpack set_i_d_ref_manual <A>
```

---

## Changelog

### 2026-04-24 — CFF trajectory: decel latch + position reference integration

#### Bug 1 — Decel phase toggled 0↔1 rapidly throughout every deceleration

**Before:** The accel/decel decision was re-evaluated from scratch every ISR:
```c
if (remaining > braking_dist + 0.05)  { phase = 0; v_cur += a*Ts; }
else                                   { phase = 1; v_cur -= a*Ts; }
```
Both `remaining` and `braking_dist = v_cur² / (2a)` change each step. When phase 1 fires and decrements `v_cur`, `braking_dist` shrinks. That shrinkage can push the condition back above the threshold, re-entering phase 0. `v_cur` then increments, braking distance grows, condition flips again — oscillating every few ISR cycles for the entire decel tail. `LOG_scurve_phase` showed rapid 0↔1 toggling in every test. The torque feedforward (which differentiates `w_m_ref`) amplified this into torque chatter.

**After:** Added `scurve_decel_latched` — a one-way flag set the first time the braking threshold is crossed and never cleared until the next `start_scurve_cff()`. Once decel starts, `v_cur` only ever decrements. `LOG_scurve_phase` now shows a clean 0 → 1 transition and stays at 1.

---

#### Bug 2 — Motor undershoots target; handoff fires with 0.5–1.5 rad remaining

**Before:** The CFF ran pure open-loop (`en_position_loop = 0`). The speed PI needs a non-zero tracking error to sustain friction torque, so the motor's actual velocity is always slightly below `scurve_v_cur`. Over a long move the motor covers less distance than the profile expects. `LOG_theta_m_ref` was not updated at all during the trajectory. At handoff the code did:
```c
LOG_theta_m_ref = scurve_target_theta;   // instant jump to final destination
en_position_loop = 1;
```
If the motor had undershot by 1.5 rad (visible in the floor 7 test at t ≈ 15 s), `LOG_theta_m_ref` jumped 1.5 rad in one ISR. The position loop saw a sudden 1.5 rad error, commanded a large speed step, the speed integrator wound up, overshot, reversed, and oscillated. This is the `LOG_T_e_cmd_inte` ringing visible in every test after the handoff.

**After:** Each ISR during the trajectory:
```c
LOG_theta_m_ref += LOG_w_m_ref * Ts;
theta_m_ref_prev  = LOG_theta_m_ref;
```
And at trajectory start, `LOG_theta_m_ref` is initialized to `LOG_theta_m_accum` (current position). The position reference now tracks the velocity integral throughout the move. At handoff the snap to `scurve_target_theta` corrects only the small speed-tracking error (typically < 0.1 rad instead of 1–2 rad). The step input to the position loop at handoff is reduced by ~15×, and the post-handoff torque integrator transient is proportionally smaller.

---

#### Bug 3 — Motor overshoots target at high speed/acceleration

**Before:** With high `a_max`, the decel ramp `v_cur -= a_max * Ts` was steep. The speed PI cannot instantly track a fast-decreasing velocity reference, so at the moment `scurve_v_cur` reached 0.05 rad/s and fired the handoff, the motor still had significant actual velocity and coasted past the target. In the high-speed floor 2 test `LOG_theta_m_error` went to −6 rad immediately at handoff, then slowly recovered as the position loop drove the motor back.

**After:** The `LOG_theta_m_ref` integration fix (Bug 2) also addresses this. The handoff velocity threshold was also raised from 0.05 to 0.3 rad/s — safe to do now that the decel latch guarantees a clean monotonic ramp, so the motor is always genuinely decelerating when this fires.

---

#### Before vs. after summary

| Symptom | Before | After |
|---------|--------|-------|
| `LOG_scurve_phase` during decel | Rapid 0↔1 toggling | Monotonic: stays at 1 |
| `LOG_theta_m_error` at handoff | Jumps 1–2 rad; −6 rad at high speed | Small step < 0.1 rad |
| `LOG_T_e_cmd_inte` after handoff | Large oscillation | Small transient, settles quickly |
| Motor overshoot (high-speed) | Passes target, reverses, oscillates | Clean decel, position loop closes small gap |
| Motor undershoot (floor 5, 7) | Handoff fires ~1.5 rad short | Handoff fires ~0 rad short |
