# Wolfpack Position Loop

## Overview
```
Ramp (proportional approach)
  └─> Position Loop (PI + VFF)
        └─> Speed Loop (PI)
              └─> MTPA decomposition
                    └─> Current Vector Limiter (8 A)
                          └─> Current Loop (synchronous-frame PI per d/q axis)
                                └─> Inverse Park → SVPWM → PWM peripheral
```

### System constants

| Constant | Value | Units |
|----------|-------|-------|
| Control rate | 10 000 | Hz |
| Encoder counts/rev | 20 000 | counts |
| Pole pairs | 4 | — |
| PM flux linkage (λ_pm) | 0.0383 | Wb |
| L_d | 1.0 | mH |
| L_q | 1.6 | mH |
| R_s | 0.55 | Ω |
| Rotor inertia (J) | 0.0042668 | kg·m² |
| Rotational damping (B) | 0.0020483 | N·m·s/rad |
| Vector current limit | 8 | A |
| OV trip | 70 | V |
| OC trip (per phase) | 12 | A |

---

## Startup Sequence

```
wolfpack init
wolfpack sm_calibrate        # motor at rest, DC off — zeros sensor offsets (~1 s)
wolfpack sm_run
wolfpack goto 10             # go to 10 rotations
wolfpack goto 0              # return to zero
wolfpack sm_idle
```

---

## Protection

Auto-latching OV and OC protection. Any trip → state TRIPPED (3), PWMs off, latched.

| Fault | Condition | Log variable |
|-------|-----------|-------------|
| Over-voltage | `LOG_v_dc > 70 V` | `LOG_OV_status_dc` |
| Over-current A | `|LOG_i_a| > 12 A` | `LOG_OC_status_a` |
| Over-current B | `|LOG_i_b| > 12 A` | `LOG_OC_status_b` |
| Over-current C | `|LOG_i_c| > 12 A` | `LOG_OC_status_c` |
| Combined | any of the above | `LOG_protection_status` |

Recover: fix root cause → `wolfpack sm_trip_clear` → `wolfpack sm_run`.

**Max safe speed: ~80 rad/s.** Back-EMF at higher speeds saturates the current regulators. Regenerative braking spikes the bus above 70 V. Field weakening is not implemented.

---

## Position Modes

### Mode 1 — Single-revolution (`set_theta_m_ref`)
- Reference wrapped to [0, 2π). Error wrapped to ±π → shortest path, never more than half a revolution.
- Feedback: `LOG_theta_m` (wrapped encoder angle).

### Mode 2 — Multi-turn absolute (`set_theta_m_ref_abs` / `goto`)
- Target is absolute angle in rad from accumulator zero (no wrapping, no shortest path).
- Feedback: `LOG_theta_m_accum` (unwrapped multi-turn).
- Runs the proportional approach ramp (see below).

### Mode 3 — Multi-turn relative (`set_theta_m_ref_rel`)
- Adds delta to the current ramp target. Stacks: two calls of 5π = one call of 10π.
- Same ramp and feedback as Mode 2.

---

## Proportional Approach Ramp

Used by `goto`, `set_theta_m_ref_abs`, and `set_theta_m_ref_rel`.

**How it works each ISR:**

```
remaining = target - ramp_integrator

if |remaining| <= snap_threshold:          # default 0.1 rad (~5.7 deg)
    ramp done
    LOG_theta_m_ref = target               # snap to exact target
    position loop closes the last gap

else:
    v_mag = gain * |remaining|             # proportional: slower as motor nears target
    v_mag = min(v_mag, w_max)             # cap at cruise speed

    v_target = +v_mag if moving forward, -v_mag if moving backward

    a_cmd = (v_target - v_prev) / Ts
    a_cmd = clamp(a_cmd, -a_max, +a_max)  # acceleration limiter

    v_new = v_prev + a_cmd * Ts
    ramp_integrator += v_new * Ts
    LOG_theta_m_ref = ramp_integrator
```

**Why proportional instead of fixed braking distance:**
A fixed braking-distance check (`v² / 2a`) causes overshoot if the factor is wrong — the motor starts braking too late and coasts past the target. The proportional approach removes this: as remaining distance → 0, commanded speed → 0. The motor cannot overshoot its own reference because the reference is always pulling toward the target, never past it.

**Snap at the end:**
When within `ramp_snap_threshold` (default 0.1 rad, ~5.7°), the ramp stops and sets `LOG_theta_m_ref = target` exactly. The position loop (which is always running in RUNNING state) closes the last few degrees cleanly. At this point the motor is already moving very slowly (gain × 0.1 rad = 0.1 rad/s at default gain), so the position loop correction is small with no reversal.

**Parameters:**

| Variable | Default | Description |
|----------|---------|-------------|
| `ramp_w_max` | 40.0 rad/s | Max cruise speed |
| `ramp_a_max` | 5.0 rad/s² | Max acceleration / deceleration |
| `ramp_approach_gain` | 1.0 1/s | Proportional gain: v = gain × remaining |
| `ramp_snap_threshold` | 0.1 rad | Distance at which ramp stops and snaps to target |

**Tuning `ramp_approach_gain`:**
- At `gain = 1.0`, speed = 1 rad/s when 1 rad away, 10 rad/s when 10 rad away.
- Ramp hits `w_max` when `remaining > w_max / gain`. For default (gain=1, w_max=40), cruise kicks in beyond 40 rad.
- Increase gain → faster approach but harder stop; decrease → slower, smoother.
- If the motor still overshoots, lower gain. If it's too slow near the target, raise gain.

---

## Position Loop

```
error       = LOG_theta_m_ref - theta_feedback
w_prop      = Kp * error
w_inte     += Ki * error * Ts    (anti-windup: skip if saturated in same direction as error)
w_vff       = Kvff * (theta_ref[k] - theta_ref[k-1]) * Fs
w_m_ref     = clamp(w_prop + w_inte + w_vff, ±pos_w_m_ref_max)
```

- `theta_feedback` = `LOG_theta_m_accum` (multi-turn modes) or `LOG_theta_m` (single-rev mode).
- VFF differentiates the position reference each ISR and adds it to the speed command. This reduces tracking lag while the ramp is moving.
- `pos_w_m_ref_max` is automatically set to `ramp_w_max` when you call `set_ramp_w_max`.

---

## Speed Loop

```
w_error    = LOG_w_m_ref - LOG_w_m_filtered
T_prop     = Kp * w_error
T_inte    += Ki * w_error * Ts    (clamped ±T_E_MAX)
T_cmd      = clamp(T_prop + T_inte, ±T_E_MAX)
```

Default gains: `Kp = J × ω_gcf = 0.02683`, `Ki = B × ω_gcf = 0.01288` at `ω_gcf = 2π rad/s`.

---

## Current Loop & MTPA

MTPA (when `mtpa_en = 1`) converts torque command to d/q current references:

```
i_s = T_cmd / (1.5 × P × λ_pm)
i_d = (λ_pm - sqrt(λ_pm² + 8·ΔL²·i_s²)) / (4·ΔL)    where ΔL = Lq - Ld
i_q = sign(i_s) × sqrt(i_s² - i_d²)
```

Current vector magnitude is clamped to 8 A before the PI regulators.

PI regulators (synchronous frame, 10 kHz):
```
v_d = Kpd × e_d + Kid × integral(e_d)
v_q = λ_pm × ω_e + Kpq × e_q + Kiq × integral(e_q)
```

---

## Logged Variables

| Variable | Description |
|----------|-------------|
| `LOG_theta_m` | Rotor angle, wrapped [0, 2π) [rad] |
| `LOG_theta_m_accum` | Rotor angle, unwrapped multi-turn [rad] |
| `LOG_theta_m_ref` | Active position reference [rad] |
| `LOG_theta_m_error` | Position error: ref − feedback [rad] |
| `LOG_theta_m_fb` | Feedback signal used by regulator [rad] |
| `LOG_w_m_ref` | Speed reference from position loop [rad/s] |
| `LOG_w_m_filtered` | Measured speed, filtered [rad/s] |
| `LOG_w_m_vff` | Velocity feedforward contribution [rad/s] |
| `LOG_ramp_theta_target` | Final ramp target [rad] |
| `LOG_ramp_theta_remaining` | Distance left to target [rad] |
| `LOG_ramp_w_allowed_prev` | Current ramp velocity command [rad/s] |
| `LOG_ramp_approach_active` | 1 = proportional zone active, 0 = at cruise speed |
| `LOG_T_e_cmd_prop` | Speed PI proportional torque [N·m] |
| `LOG_T_e_cmd_inte` | Speed PI integral torque [N·m] |
| `LOG_T_e_cmd` | Total torque command [N·m] |
| `LOG_i_d`, `LOG_i_q` | d/q currents [A] |
| `LOG_v_dc` | DC bus voltage [V] |
| `LOG_OV_status_dc` | 1 if OV trip active |
| `LOG_OC_status_a/b/c` | 1 if OC trip active per phase |
| `LOG_wolf_state` | State: 0=CALIBRATE, 1=IDLE, 2=RUNNING, 3=TRIPPED |

---

## Full Command Reference

```
wolfpack init
wolfpack deinit
wolfpack sm_get_state
wolfpack sm_calibrate
wolfpack sm_run
wolfpack sm_idle
wolfpack sm_trip_clear

wolfpack goto <rotations>                  # ramp to absolute position (multi-turn)
wolfpack set_theta_m_ref <rad>             # single-rev position, shortest path
wolfpack set_theta_m_ref_abs <rad>         # multi-turn absolute position (same as goto but in rad)
wolfpack set_theta_m_ref_rel <delta_rad>   # add delta to current ramp target
wolfpack set_w_m_ref <rad/s>              # direct speed command (bypasses position loop)

wolfpack set_ramp_w_max <rad/s>            # cruise speed limit (also sets pos loop clamp)
wolfpack set_ramp_a_max <rad/s²>           # acceleration limit
wolfpack set_ramp_approach_gain <1/s>      # proportional approach gain (default 1.0)
wolfpack set_ramp_snap_threshold <rad>     # snap distance (default 0.1 rad = 5.7 deg)

wolfpack set_pos_kp <val>
wolfpack set_pos_ki <val>
wolfpack set_pos_vff_gain <val>

wolfpack set_speed_kp <val>
wolfpack set_speed_ki <val>

wolfpack set_ireg_kpd <val>
wolfpack set_ireg_kid <val>
wolfpack set_ireg_kpq <val>
wolfpack set_ireg_kiq <val>
wolfpack set_mtpa_en <0|1>

wolfpack set_i_q_ref_manual <A>
wolfpack set_i_d_ref_manual <A>
```
