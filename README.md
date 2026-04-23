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

---

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
wolfpack set_theta_m_ref 0        # go to encoder zero
wolfpack set_theta_m_ref 1.5708   # go to 90° (CW from zero)
wolfpack set_theta_m_ref 4.7124   # go to 270° — motor takes shortest path (could go CW or CCW)
```

> **Note:** `set_theta_m_ref 7.0` and `set_theta_m_ref 0.717` command the same physical position because 7.0 wraps to 7.0 − 2π ≈ 0.717 rad.

---

### Mode 2 — Multi-turn absolute (`set_theta_m_ref_abs`)

```
wolfpack set_theta_m_ref_abs <rad>
```

- Reference is the **true absolute angle from encoder power-on zero**, with no wrapping.
- Feedback is `LOG_theta_m_accum` — the unwrapped accumulated angle that increases without bound as the shaft turns.
- **No shortest-path routing.** The motor goes exactly to the commanded absolute angle. If you command 10π (5 full turns CW), it goes 5 full turns CW.
- Use this when direction and absolute position matter — robot joints, lead screws, precise multi-turn positioning.

**Example:**

```
wolfpack set_theta_m_ref_abs 0        # go to encoder power-on zero
wolfpack set_theta_m_ref_abs 1.5708   # go to exactly 90° CW from zero
wolfpack set_theta_m_ref_abs 31.416   # go to exactly 5 full turns CW from zero (10π rad)
wolfpack set_theta_m_ref_abs -3.1416  # go to exactly half a turn CCW from zero
```

---

### Mode 3 — Multi-turn relative (`set_theta_m_ref_rel`)

```
wolfpack set_theta_m_ref_rel <delta_rad>
```

- **Adds** `delta_rad` to the current position reference.
- On the first call (or after switching from another mode), the reference is anchored to the current shaft position (`LOG_theta_m_accum`) before the delta is applied.
- Subsequent calls stack — two calls of 5π reach the same endpoint as one call of 10π.
- Feedback is `LOG_theta_m_accum`.

**Example:**

```
# Motor is at 0 rad (power-on zero)
wolfpack set_theta_m_ref_rel 3.1416   # move +π rad CW from current position
wolfpack set_theta_m_ref_rel 3.1416   # move another +π rad CW — now at 2π total
wolfpack set_theta_m_ref_rel -6.2832  # move 2π rad CCW — back to 0
```

**Stacking example — all three reach the same endpoint:**

```
wolfpack set_theta_m_ref_rel 31.416           # one call: 10π
wolfpack set_theta_m_ref_rel 15.708  (×2)     # two calls: 5π + 5π
wolfpack set_theta_m_ref_rel 6.2832  (×5)     # five calls: 2π × 5
```

> **Note:** `set_theta_m_ref_rel` adds to the **reference**, not the current shaft position. If the motor has not yet reached its previous target when you issue a new relative command, the new delta is added to wherever the reference currently is, not where the shaft is. For sequential absolute moves, use `set_theta_m_ref_abs` instead.

---

## How the Position Loop Works

### Proportional + Velocity Feedforward

```
error       = theta_ref − theta_feedback
w_ref_prop  = Kpp × error
w_vff       = Kvff × (theta_ref[k] − theta_ref[k−1]) / Ts
w_ref       = w_ref_prop + w_vff
w_ref       = clamp(w_ref, ±w_ref_max)
```

The **proportional term** reacts to position error. Without feedforward, the motor always lags behind a moving reference because it can only generate speed once error has built up.

The **velocity feedforward** differentiates the position reference each ISR cycle (10 kHz) and adds it directly to the speed reference. When the reference is changing (e.g., stacked relative moves), the motor gets a pre-emptive speed command proportional to how fast the reference is moving — before any error accumulates. For a step command from the CLI, the feedforward fires a one-cycle impulse that is absorbed by the `w_ref_max` clamp, so no harm is done.

### Speed Clamp

The output of the position loop is clamped to `±pos_w_m_ref_max` (default 40 rad/s). This limits how aggressively the motor chases a position, preventing the speed loop from being over-driven.

---

## How the Speed Loop Works

### PI + CFF Torque Feedforward

```
w_error    = w_ref − w_m_filtered
T_prop     = Kpv × w_error
T_inte    += Kiv × w_error × Ts                   (clamped ±T_E_MAX, anti-windup)
T_ff       = speed_vff_gain × J × (w_ref[k] − w_ref[k−1]) × 10000
T_cmd      = T_prop + T_inte + T_ff
```

`T_ff = J × dω*/dt` is exactly the torque needed to accelerate the rotor inertia at the commanded rate. It is supplied before any error accumulates. The PI integrator then handles only friction, load disturbances, and model error. Set `speed_vff_gain = 0` to disable CFF and observe `LOG_T_e_cmd_inte` wind up during moves to see the difference.

### Anti-windup

The speed integrator is clamped to `±T_E_MAX = 1.5 × P × λ_pm × I_lim ≈ ±0.46 N·m`. This is the maximum torque the current limiter will physically allow. The integrator cannot wind past this bound even when the motor is blocked or tracking a large step command.

---

## Tuning Parameters

### Position loop gains

| Command | Default | Description |
|---|---|---|
| `set_pos_kp <val>` | 12.0 | Position proportional gain [rad/s per rad]. Higher = faster response, more overshoot risk. |
| `set_pos_ki <val>` | 20.0 | Position integral gain [rad/s per rad*s]. Removes steady-state position offset. |
| `set_pos_w_m_ref_max <val>` | 40.0 | Speed command clamp [rad/s]. Limits how fast the motor chases position. |
| `set_pos_vff_gain <val>` | 1.0 | Velocity feedforward gain. 1.0 = full feedforward, 0.0 = off. Reduce if following a smooth trajectory and you observe overshoot. |

### Speed loop gains

| Command | Default | Description |
|---|---|---|
| `set_speed_kp <val>` | J × ω_gcf | Speed PI proportional gain [N·m·s/rad]. |
| `set_speed_ki <val>` | B × ω_gcf | Speed PI integral gain [N·m/rad]. |
| `set_speed_vff_gain <val>` | 1.0 | CFF torque feedforward gain. 1.0 = full inertia compensation, 0.0 = off. |

### Tuning procedure

1. Start with velocity feedforward **disabled**: `set_pos_vff_gain 0`.
2. Tune `set_pos_kp`, `set_pos_ki`, and the speed loop gains until the step response is acceptably damped and reaches zero steady-state error.
3. Re-enable velocity feedforward: `set_pos_vff_gain 1.0`. Observe reduction in position tracking lag on moving references.
4. Re-enable velocity feedforward: `set_pos_vff_gain 1.0`. Observe reduction in position tracking lag on moving references. If overshoot increases on step commands, this is normal — the FF is absorbed by the clamp and does not cause steady overshoot.

---

## Logged Variables

These variables are available for logging (e.g., via the AMDC host interface):

| Variable | Description |
|---|---|
| `LOG_theta_m` | Rotor angle, wrapped to [0, 2π) [rad] |
| `LOG_theta_m_accum` | Rotor angle, unwrapped multi-turn absolute [rad] |
| `LOG_theta_m_ref` | Active position reference [rad] |
| `LOG_theta_m_error` | Position error: ref − feedback [rad] |
| `LOG_w_m_ref` | Speed reference output of position loop (post-clamp) [rad/s] |
| `LOG_w_m_vff` | Velocity feedforward contribution to speed reference [rad/s] |
| `LOG_T_e_cmd_prop` | Speed PI proportional torque command [N·m] |
| `LOG_T_e_cmd_inte` | Speed PI integral torque command [N·m] |
| `LOG_T_e_cmd` | Total torque command [N·m] |
| `LOG_scurve_progress` | CFF trajectory normalized progress 0.0 → 1.0 |
| `LOG_scurve_t_elapsed` | CFF trajectory elapsed time [s] |
| `LOG_scurve_t_total` | CFF trajectory total planned duration [s] |
| `LOG_T_e_cmd_ff` | CFF torque feedforward contribution [N·m] |

---

## CFF Velocity Trajectory (Elevator Mode)

Instead of stepping the position reference to a new floor instantly (which forces the speed PI to react from a large error), the CFF trajectory:

1. Generates a **jerk-limited velocity profile** (raised-cosine shape) — the motor speeds up, optionally cruises at `v_max`, then decelerates smoothly to zero.
2. Adds a **torque feedforward** term each ISR derived from the commanded acceleration — this pre-supplies the inertia torque so the PI integrator does not need to wind up.

This is the Command Feedforward (CFF) approach from ME 547 (Lorenz): `T_ff = J × dω*/dt`.

---

### Signal flow

```
  wolfpack scurve_goto 30   ← user says "go to Floor 3 (30 rotations)"
              │
              ▼
  ┌─────────────────────┐   called once
  │  start_scurve_cff() │─────────────────────────────────────────────┐
  │  • target = 30×2π   │   computes t_accel, t_const, t_total        │
  │  • direction = +1   │   sets scurve_active = 1                    │
  └─────────────────────┘   en_position_loop = 0 (velocity mode)      │
                                                                       │
  ─────────── every ISR at 10 kHz ───────────────────────────────────  │
              │                                                        │
              ▼                                                        │
  ┌─────────────────────┐                                             │
  │  update_scurve_cff()│  raised-cosine velocity profile             │
  │  sets LOG_w_m_ref(t)│  ← this is ω*(t)                           │
  └──────────┬──────────┘                                             │
             │                                                        │
      ┌──────┴─────────────────────────┐                              │
      │                                │                              │
      ▼                                ▼                              │
  ┌──────────────────┐    ┌────────────────────────────┐             │
  │   Speed PI       │    │   CFF Torque Feedforward   │             │
  │   T_prop + T_inte│    │   T_ff = J × Δω*/Ts        │             │
  │   (tracks error) │    │   (inertia compensation)   │             │
  └────────┬─────────┘    └──────────────┬─────────────┘             │
           └──────────────┬──────────────┘                           │
                          │                                           │
                          ▼                                           │
                    LOG_T_e_cmd                                       │
                          │                                           │
                          ▼                                           │
               Current Loop (MTPA + PI)                              │
                          │                                           │
                          ▼                                           │
                    SVPWM → Motor                                     │
                          │                                           │
                          ▼                                           │
             when |target − θ_accum| < 0.5 rad:                      │
  ┌─────────────────────────────────────┐                            │
  │  position hold handoff              │◄───────────────────────────┘
  │  scurve_active = 0                  │
  │  en_position_loop = 1               │
  │  LOG_theta_m_ref = target           │
  └─────────────────────────────────────┘
```

---

### Numerical example: Floor 1 → Floor 3 (30 rotations)

**Command:**
```
wolfpack scurve_en 1
wolfpack scurve_goto 30
```

#### Step 1 — Trajectory planning (`start_scurve_cff`, runs once)

```
target_theta = 30 × 2π         =  188.5 rad
delta        = |188.5 − 0|     =  188.5 rad   (motor starts at 0)

t_accel  = v_max/a_max + a_max/j_max
         = 18/2.5   +   2.5/25
         =  7.2     +   0.1    =   7.3 s

v_max × t_accel = 18 × 7.3    = 131.4 rad

delta (188.5) > 131.4  →  long move, cruise phase exists

t_const  = (188.5 − 131.4) / 18  =   3.2 s
t_total  =  2 × 7.3  +  3.2      =  17.8 s
direction = +1  (target > current)
```

#### Step 2 — Velocity profile (`update_scurve_cff`, every 100 µs)

The velocity follows a raised-cosine from 0 → `v_max` → 0 over `t_total`:

```
  ω* [rad/s]
  18 ──────────────────────────────────
     /                                  \
  9 /                                    \
   /                                      \
  0──────────────────────────────────────── t [s]
  0    3.6    7.3   8.9  10.5   14.2   17.8

  │ accel │ cruise (3.2 s) │ decel  │
```

| Time [s] | Progress | `LOG_w_m_ref` [rad/s] | Phase |
|----------|----------|----------------------|-------|
| 0        | 0.00     | 0                    | start |
| 3.6      | 0.20     | 9.0                  | accelerating |
| 7.3      | 0.41     | 18.0                 | at v_max |
| 8.9      | 0.50     | 18.0                 | cruise midpoint |
| 10.5     | 0.59     | 18.0                 | start decel |
| 14.2     | 0.80     | 9.0                  | decelerating |
| 17.8     | 1.00     | 0                    | done → pos hold |

#### Step 3 — CFF torque feedforward (every ISR, alongside speed PI)

```
T_ff = speed_vff_gain × J_ESTIMATE × (ω*[k] − ω*[k−1]) × 10 000
     = 1.0 × 0.00427 × Δω* × 10 000

Peak acceleration occurs at t ≈ 3.6 s (progress = 0.25):
  dω*/dt  = v_max × π / t_total × sin(2π × 0.25)
           = 18 × π / 17.8 × 1.0
           ≈ 3.18 rad/s²

  T_ff_peak  = J × dω*/dt
             = 0.00427 × 3.18
             ≈ 0.014 N·m   ← pre-supplied inertia torque

During cruise (constant velocity):
  dω*/dt = 0  →  T_ff = 0   ← PI handles friction/load only

During decel (mirror of accel):
  T_ff  ≈ −0.014 N·m        ← braking assist
```

**The key benefit:** `LOG_T_e_cmd_inte` stays near zero during the move. The integrator only handles friction and load disturbances; the inertia torque is already supplied by CFF.

#### Step 4 — Position handoff

At `t ≈ 17.3 s`, `|188.5 − LOG_theta_m_accum| < 0.5 rad`:
- `scurve_active = 0`
- `en_position_loop = 1`, `LOG_theta_m_ref = 188.5 rad`
- Position PI takes over for the final precision stop

---

### Commands

```
wolfpack scurve_en <0|1>             # 0 = direct, 1 = route through CFF trajectory
wolfpack scurve_set_v_max <rad/s>    # max velocity  (default 18.0)
wolfpack scurve_set_a_max <rad/s²>   # max accel     (default 2.5)
wolfpack scurve_set_j_max <rad/s³>   # max jerk      (default 25.0)
wolfpack scurve_goto <rotations>     # CFF move to absolute rotation target
wolfpack scurve_goto_vel <rotations> # same as scurve_goto (alias)
wolfpack scurve_stop                 # abort trajectory, hold current position
wolfpack set_speed_vff_gain <val>    # CFF torque FF gain (default 1.0 = full, 0 = off)
```

When `scurve_en 1`, `set_theta_m_ref_abs` and `set_theta_m_ref_rel` automatically route through the CFF trajectory. `set_theta_m_ref` (single-rev wrapped mode) is unaffected.

### Typical elevator workflow

```
wolfpack scurve_en 1

wolfpack scurve_goto 10    # Floor 1  (10 rotations)
wolfpack scurve_goto 30    # Floor 3  (30 rotations)
wolfpack scurve_goto 0     # Ground floor

# To compare with CFF off:
wolfpack set_speed_vff_gain 0     # PI alone — watch LOG_T_e_cmd_inte wind up
wolfpack set_speed_vff_gain 1     # CFF back on
```

### Effect of `speed_vff_gain`

| Setting | Behavior |
|---------|----------|
| `1.0` (default) | Full inertia compensation — integrator stays small during moves |
| `0.5` | Partial — useful if `J_ESTIMATE` is too high |
| `0.0` | CFF off — pure PI, integrator must supply all torque |

### Conservative starting parameters

| Parameter | Conservative | Normal | Aggressive |
|-----------|-------------|--------|------------|
| v_max [rad/s]  | 12–15  | 18–22  | 28–35 |
| a_max [rad/s²] | 1.5–2  | 2.5–3.5 | 4–6  |
| j_max [rad/s³] | 15–20  | 25–35  | 40–60 |

Always start conservative and increase while watching `LOG_i_q` for saturation.

---

## Startup Sequence

```
wolfpack init
wolfpack sm_calibrate      # zero current/voltage sensor offsets (~1 second)
wolfpack sm_run            # enable PWMs, enter running state

wolfpack set_theta_m_ref_abs 0        # go to encoder zero
wolfpack set_theta_m_ref_abs 1.5708   # rotate 90° CW
wolfpack set_theta_m_ref_rel 3.1416   # rotate another 180° CW from current position

wolfpack sm_idle           # disable PWMs, hold position reference
```

---

## State Machine Behavior

When the motor transitions to **IDLE**:
- All integrators are zeroed (current, speed).
- `LOG_theta_m_ref` is frozen at `LOG_theta_m_accum` (current shaft position) — no jump when returning to RUN.
- Feedforward state (`theta_m_ref_prev`, `LOG_w_m_ref_prev`) is synced to avoid transients on re-entry.
- The IDLE reset happens **once on entry**, not every cycle, so position commands issued while in IDLE are preserved and take effect immediately when `sm_run` is called.

When transitioning between position/speed modes:
- The speed integrator is always reset to zero on any mode switch (`set_w_m_ref`, `set_theta_m_ref`, `set_theta_m_ref_abs`, `set_theta_m_ref_rel`) to prevent torque bumps.
