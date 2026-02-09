#ifndef TASK_WOLFPACK_H
#define TASK_WOLFPACK_H

#include "sys/scheduler.h"

#define TASK_WOLFPACK_UPDATES_PER_SEC (10000)
#define TASK_WOLFPACK_INTERVAL_USEC   (USEC_IN_SEC / TASK_WOLFPACK_UPDATES_PER_SEC)
#define DEADTIME_MINIMUM_NS (400)		// Minimum deadtime between Wolfpack gate commands


int task_wolfpack_init(void);
int task_wolfpack_deinit(void);

void task_wolfpack_callback(void *arg);

double compute_log_dt(double current);
void task_controller_stats_print(void);
void task_controller_stats_reset(void);

#endif // TASK_WOLFPACK_H
