#ifndef TRAJ_SCURVE_H
#define TRAJ_SCURVE_H

// S-curve (jerk-limited) trajectory generator.
// Pre-computes all 7 phase durations at move-start; integrates pos/vel/accel
// each ISR call.  Handles any distance: cruise phase collapses for short moves.
// Uses float internally (FPU-accelerated on Cortex-A9).
// Supports asymmetric up/down limits for gravity-compensated axes.

void   traj_scurve_init(void);

// Set motion limits for upward (+) direction
void   traj_scurve_set_jmax(float jmax);    // [rad/s³]
void   traj_scurve_set_amax(float amax);    // [rad/s²]
void   traj_scurve_set_vmax(float vmax);    // [rad/s]

// Set motion limits for downward (-) direction (defaults equal to up limits)
void   traj_scurve_set_jmax_dn(float jmax);
void   traj_scurve_set_amax_dn(float amax);
void   traj_scurve_set_vmax_dn(float vmax);

// Start a move.  pos_current = present accumulated motor angle [rad].
// theta_target = destination in same frame (LOG_theta_m_accum space).
void   traj_scurve_move_abs(double pos_current, double theta_target);

// Called every ISR — integrates one Ts step and returns new position ref [rad].
// When trajectory finishes, snaps to exact target and marks inactive.
double traj_scurve_update(void);

int    traj_scurve_is_active(void);
int    traj_scurve_get_phase(void);      // 0 = idle, 1-7 = active phase
float  traj_scurve_get_progress(void);   // 0.0-1.0 (distance fraction)
double traj_scurve_get_end_pos(void);    // target position [rad]
void   traj_scurve_abort(void);

#endif // TRAJ_SCURVE_H
