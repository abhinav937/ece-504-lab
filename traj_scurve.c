
#include "traj_scurve.h"
#include <math.h>

#define TRAJ_TS      (1.0f / 10000.0f)   // 100 µs — must match TASK_WOLFPACK_UPDATES_PER_SEC
#define TRAJ_D_MIN   (1e-4f)              // moves shorter than this are skipped

// Default motion limits
#define DEFAULT_JMAX (500.0f)    // [rad/s³]
#define DEFAULT_AMAX (20.0f)     // [rad/s²]
#define DEFAULT_VMAX (10.0f)     // [rad/s]

// Jerk sign for each phase (1-indexed; [0] unused)
static const float JERK_SIGN[8] = { 0.0f, 1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 1.0f };

// Expected acceleration at the START of each phase — used to snap on transition.
// Stored as multiples of a_eff; filled in _compute_profile.
// Phases: 1→0, 2→+a, 3→+a, 4→0, 5→0, 6→-a, 7→-a
static const float ACCEL_SNAP_SIGN[8] = { 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, -1.0f, -1.0f };

typedef struct {
    // Stored limits
    float j_up, a_up, v_up;
    float j_dn, a_dn, v_dn;

    // Profile parameters for the current move
    float j_eff;        // jerk used (from up/dn set)
    float a_eff;        // effective acceleration (may be clamped below a_limit)
    float T[8];         // phase durations [s], T[0] unused

    // Integration state (all in the positive-direction frame: 0..d_total)
    float pos;          // integrated distance from move start [rad, >=0]
    float vel;          // [rad/s, >=0 during accel, <0 during decel relative to profile]
    float accel;        // [rad/s²]
    float d_total;      // total move distance [rad, >0]

    // Reference frame
    double pos_start;   // starting accumulated angle [rad]
    double theta_target;

    float dir;          // +1 or -1
    int   phase;        // 0=idle, 1-7=active
    float t_phase;      // elapsed time in current phase [s]
    int   active;
} traj_state_t;

static traj_state_t s;

// ---------------------------------------------------------------------------
// Profile pre-computation
// ---------------------------------------------------------------------------
// Given total distance d [rad] and limits j/a/v, computes s.T[1..7] and s.a_eff.
// Derivation:
//   d_accel(T2) = a³/j² + (3a²/2j)*T2 + 0.5*a*T2²   (one half of symmetric profile)
//   Full cruise:  T4 = (d - 2*d_accel(T2_full)) / v,  T2_full = v/a - t_j
//   No cruise:    solve 2*d_accel(T2_new) = d  →  T2_new = (-3a/j + sqrt(a²/j² + 4d/a)) / 2
//   Very short:   d < 2a³/j²  →  T2=T6=T4=0, reduce a to cbrt-like formula
// ---------------------------------------------------------------------------
static void _compute_profile(float d, float j, float a, float v)
{
    // Clamp a so the jerk ramps alone don't overshoot v (prevents negative T2)
    float a_lim = sqrtf(v * j);
    if (a > a_lim) a = a_lim;
    s.a_eff = a;

    float t_j = a / j;

    // Minimum distance achievable with T2=T4=T6=0 at this a
    float d_full_jerk = 2.0f * a * a * a / (j * j);

    if (d < d_full_jerk) {
        // Very short move: T2=T4=T6=0, must also reduce a (and thus v_peak)
        // d = 2*a_new³/j²  →  a_new = (j²*d/2)^(1/3)
        a = cbrtf(j * j * d / 2.0f);
        s.a_eff = a;
        t_j = a / j;
        s.T[1] = t_j; s.T[2] = 0.0f; s.T[3] = t_j;
        s.T[4] = 0.0f;
        s.T[5] = t_j; s.T[6] = 0.0f; s.T[7] = t_j;
        return;
    }

    // T2 for reaching v_max with this a
    float T2_full = v / a - t_j;   // >= 0 because a <= sqrt(v*j)

    // d_accel = a³/j² + (3a²/2j)*T2 + 0.5*a*T2²
    float d_accel_full = a * a * a / (j * j)
                       + 3.0f * a * a / (2.0f * j) * T2_full
                       + 0.5f * a * T2_full * T2_full;

    if (d >= 2.0f * d_accel_full) {
        // Full S-curve with cruise phase
        s.T[1] = t_j;     s.T[2] = T2_full; s.T[3] = t_j;
        s.T[4] = (d - 2.0f * d_accel_full) / v;
        s.T[5] = t_j;     s.T[6] = T2_full; s.T[7] = t_j;
    } else {
        // T4=0 — solve quadratic: T2² + (3a/j)*T2 + (2a²/j² - d/a) = 0
        // T2 = (-3a/j + sqrt(a²/j² + 4d/a)) / 2
        float disc = a * a / (j * j) + 4.0f * d / a;
        float T2_new = (-3.0f * a / j + sqrtf(disc)) / 2.0f;
        if (T2_new < 0.0f) T2_new = 0.0f;
        s.T[1] = t_j;    s.T[2] = T2_new; s.T[3] = t_j;
        s.T[4] = 0.0f;
        s.T[5] = t_j;    s.T[6] = T2_new; s.T[7] = t_j;
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void traj_scurve_init(void)
{
    s.j_up = DEFAULT_JMAX; s.a_up = DEFAULT_AMAX; s.v_up = DEFAULT_VMAX;
    s.j_dn = DEFAULT_JMAX; s.a_dn = DEFAULT_AMAX; s.v_dn = DEFAULT_VMAX;
    s.phase  = 0;
    s.active = 0;
    s.pos = s.vel = s.accel = 0.0f;
    s.t_phase = 0.0f;
    s.d_total = 0.0f;
    s.dir = 1.0f;
    s.pos_start = 0.0;
    s.theta_target = 0.0;
}

void traj_scurve_set_jmax(float jmax) { if (jmax > 0.0f) s.j_up = jmax; }
void traj_scurve_set_amax(float amax) { if (amax > 0.0f) s.a_up = amax; }
void traj_scurve_set_vmax(float vmax) { if (vmax > 0.0f) s.v_up = vmax; }

void traj_scurve_set_jmax_dn(float jmax) { if (jmax > 0.0f) s.j_dn = jmax; }
void traj_scurve_set_amax_dn(float amax) { if (amax > 0.0f) s.a_dn = amax; }
void traj_scurve_set_vmax_dn(float vmax) { if (vmax > 0.0f) s.v_dn = vmax; }

void traj_scurve_move_abs(double pos_current, double theta_target)
{
    double delta = theta_target - pos_current;

    if (fabsf((float)delta) < TRAJ_D_MIN) {
        // Already at target — nothing to do
        return;
    }

    s.dir          = (delta > 0.0) ? 1.0f : -1.0f;
    s.pos_start    = pos_current;
    s.theta_target = theta_target;
    s.d_total      = (float)fabs(delta);

    // Pick limits based on direction
    float j = (s.dir > 0.0f) ? s.j_up : s.j_dn;
    float a = (s.dir > 0.0f) ? s.a_up : s.a_dn;
    float v = (s.dir > 0.0f) ? s.v_up : s.v_dn;

    _compute_profile(s.d_total, j, a, v);

    // Reset integration state
    s.pos     = 0.0f;
    s.vel     = 0.0f;
    s.accel   = 0.0f;
    s.phase   = 1;
    s.t_phase = 0.0f;
    s.active  = 1;
}

double traj_scurve_update(void)
{
    if (!s.active) {
        return s.theta_target;
    }

    // Skip zero-duration phases instantly
    while (s.phase <= 7 && s.T[s.phase] <= 0.0f) {
        // Snap accel to expected value at start of this phase
        s.accel = ACCEL_SNAP_SIGN[s.phase] * s.a_eff;
        s.phase++;
    }

    if (s.phase > 7) {
        // Trajectory complete — clamp to exact target
        s.pos    = s.d_total;
        s.vel    = 0.0f;
        s.accel  = 0.0f;
        s.active = 0;
        s.phase  = 0;
        return s.theta_target;
    }

    // Integrate one Ts step
    float j_cmd = JERK_SIGN[s.phase] * s.j_eff;
    s.accel   += j_cmd  * TRAJ_TS;
    s.vel     += s.accel * TRAJ_TS;
    s.pos     += s.vel   * TRAJ_TS;

    // Advance phase timer; transition if phase elapsed
    s.t_phase += TRAJ_TS;
    if (s.t_phase >= s.T[s.phase]) {
        s.t_phase -= s.T[s.phase];
        s.phase++;
        // Snap accel to exact expected value at start of next phase to prevent drift
        if (s.phase <= 7) {
            s.accel = ACCEL_SNAP_SIGN[s.phase] * s.a_eff;
        }
    }

    // Safety clamp: never overshoot the target distance
    if (s.pos > s.d_total) s.pos = s.d_total;

    return s.pos_start + (double)(s.dir * s.pos);
}

int traj_scurve_is_active(void)
{
    return s.active;
}

int traj_scurve_get_phase(void)
{
    return s.phase;
}

float traj_scurve_get_progress(void)
{
    if (s.d_total <= 0.0f) return 1.0f;
    float p = s.pos / s.d_total;
    if (p > 1.0f) p = 1.0f;
    return p;
}

double traj_scurve_get_end_pos(void)
{
    return s.theta_target;
}

void traj_scurve_abort(void)
{
    s.active = 0;
    s.phase  = 0;
    s.vel    = 0.0f;
    s.accel  = 0.0f;
}
