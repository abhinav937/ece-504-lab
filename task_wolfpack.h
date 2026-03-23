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

int task_wolfpack_set_en_speed_loop(int v);
int task_wolfpack_set_en_mtpa(int v);
int task_wolfpack_set_en_fw(int v);
int task_wolfpack_set_en_state_fb(int v);
int task_wolfpack_set_en_current_loop(int v);
int task_wolfpack_set_en_vector_limit(int v);

void task_wolfpack_stats_print(void);
void task_wolfpack_stats_reset(void);

#endif // TASK_WOLFPACK_H
