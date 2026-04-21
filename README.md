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

### PI + Acceleration Feedforward

```
w_error    = w_ref − w_m_filtered
T_prop     = Kpv × w_error
T_inte    += Kiv × w_error × Ts          (clamped to ±T_E_MAX for anti-windup)
T_aff      = Kaff × (w_ref[k] − w_ref[k−1]) / Ts
T_cmd      = T_prop + T_inte + T_aff
```

The **acceleration feedforward** differentiates the speed reference and scales by `Kaff` (default = `J_ESTIMATE`, the rotor inertia in kg·m²). This gives `T_aff = J × dω*/dt` — exactly the torque needed to physically accelerate the rotor at the commanded rate. The speed PI integrator does not need to wind up to find this torque; it is already supplied by feedforward. The integrator then handles only steady-state disturbances (friction, load, etc.).

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
