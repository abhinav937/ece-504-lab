#ifndef TASK_WOLFPACK_H
#define TASK_WOLFPACK_H

#include "sys/scheduler.h"

#define TASK_WOLFPACK_UPDATES_PER_SEC (10000)
#define TASK_WOLFPACK_INTERVAL_USEC   (USEC_IN_SEC / TASK_WOLFPACK_UPDATES_PER_SEC)
#define DEADTIME_MINIMUM_NS (400)		// Minimum deadtime between Wolfpack gate commands


int task_wolfpack_init(void);
int task_wolfpack_deinit(void);

void task_wolfpack_callback(void *arg);

void task_wolfpack_sm_run(void);
void task_wolfpack_sm_idle(void);
void task_wolfpack_sm_calibrate(void);
void task_wolfpack_sm_trip_clear(void);
int task_wolfpack_sm_get_state(void);

int task_wolfpack_set_i_q_ref_manual(double i);
int task_wolfpack_set_i_d_ref_manual(double i);
int task_wolfpack_set_w_m_ref(double w);
int task_wolfpack_set_theta_m_ref(double theta);
int task_wolfpack_set_theta_m_ref_abs(double theta);
int task_wolfpack_set_theta_m_ref_rel(double delta_theta);
int task_wolfpack_set_pos_kp(double kp);
int task_wolfpack_set_pos_ki(double ki);
int task_wolfpack_set_pos_w_m_ref_max(double w_max);
int task_wolfpack_set_mtpa_en(int en);
int task_wolfpack_set_pos_vff_gain(double gain);
int task_wolfpack_set_speed_kp(double kp);
int task_wolfpack_set_speed_ki(double ki);
int task_wolfpack_set_ireg_kpd(double kp);
int task_wolfpack_set_ireg_kid(double ki);
int task_wolfpack_set_ireg_kpq(double kp);
int task_wolfpack_set_ireg_kiq(double ki);

void task_wolfpack_stats_print(void);
void task_wolfpack_stats_reset(void);

// S-curve enable/disable and parameter setters
int task_wolfpack_set_scurve_en(int en);
int task_wolfpack_set_scurve_v_max(double v);
int task_wolfpack_set_scurve_a_max(double a);
int task_wolfpack_set_scurve_j_max(double j);
int task_wolfpack_scurve_stop(void);

// S-curve goto helpers (use stored default params)
void task_wolfpack_scurve_goto(double target_rotations);
void task_wolfpack_scurve_goto_vel(double target_rotations);

// CFF trajectory starter (explicit, bypasses scurve_en flag)
void task_wolfpack_start_scurve_cff(double target_rotations,
                                     double v_max, double a_max, double j_max);

// CFF torque feedforward gain
int task_wolfpack_set_speed_vff_gain(double gain);


#endif // TASK_WOLFPACK_H
