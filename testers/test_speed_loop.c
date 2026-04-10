#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TASK_WOLFPACK_UPDATES_PER_SEC (10000.0)
#define TS (1.0 / TASK_WOLFPACK_UPDATES_PER_SEC)

#define POLE_PAIRS (4.0)
#define PM_FLUX_V_SEC_PER_RAD (0.0383)
#define L_DS_ESTIMATE (0.001)
#define L_QS_ESTIMATE (0.0016)
#define VECTOR_CURRENT_LIMIT (2.0)

#define J_ESTIMATE (0.0042668)
#define B_ESTIMATE (0.0020483)
#define SPEED_REG_W_GCF (6.28318530718)
#define KP_W_M (J_ESTIMATE * SPEED_REG_W_GCF)
#define KI_W_M (B_ESTIMATE * SPEED_REG_W_GCF)

#define SIM_TIME_SEC (4.0)
#define TRACKING_TOL_RAD_PER_SEC (2.0)

typedef struct {
    double i_d;
    double i_q;
} dq_current_t;

typedef struct {
    const char *name;
    double w_ref;
    double load_torque;
    double initial_speed;
    double final_speed_min;
    double final_speed_max;
} speed_case_t;

static double torque_from_dq(double i_d, double i_q)
{
    return 1.5 * POLE_PAIRS *
           (PM_FLUX_V_SEC_PER_RAD * i_q + i_q * i_d * (L_DS_ESTIMATE - L_QS_ESTIMATE));
}

static dq_current_t compute_mtpa(double i_s_ref)
{
    dq_current_t out = {0.0, 0.0};
    double delta_L = L_QS_ESTIMATE - L_DS_ESTIMATE;

    if (fabs(delta_L) > 1e-12) {
        double mtpa_radicand = PM_FLUX_V_SEC_PER_RAD * PM_FLUX_V_SEC_PER_RAD +
                               8.0 * delta_L * delta_L * i_s_ref * i_s_ref;
        out.i_d = (PM_FLUX_V_SEC_PER_RAD - sqrt(mtpa_radicand)) / (4.0 * delta_L);
    }

    {
        double iq_mtpa_radicand = i_s_ref * i_s_ref - out.i_d * out.i_d;
        out.i_q = (i_s_ref >= 0.0 ? 1.0 : -1.0) * sqrt(fmax(0.0, iq_mtpa_radicand));
    }

    return out;
}

static dq_current_t limit_current_vector(dq_current_t ref)
{
    double mag = sqrt(ref.i_d * ref.i_d + ref.i_q * ref.i_q);
    if (mag > VECTOR_CURRENT_LIMIT && mag > 0.0) {
        double scale = VECTOR_CURRENT_LIMIT / mag;
        ref.i_d *= scale;
        ref.i_q *= scale;
    }
    return ref;
}

static int run_case(const speed_case_t *tc)
{
    int steps = (int)(SIM_TIME_SEC * TASK_WOLFPACK_UPDATES_PER_SEC);
    double w_m = tc->initial_speed;
    double integ = 0.0;
    double peak_speed = w_m;
    double final_error;

    for (int k = 0; k < steps; ++k) {
        double w_err = tc->w_ref - w_m;
        double t_cmd_prop = KP_W_M * w_err;
        integ += KI_W_M * w_err * TS;
        double t_cmd = t_cmd_prop + integ;

        double i_s_ref = t_cmd / (1.5 * POLE_PAIRS * PM_FLUX_V_SEC_PER_RAD);
        dq_current_t i_mtpa = compute_mtpa(i_s_ref);
        dq_current_t i_cmd = limit_current_vector(i_mtpa);

        /* Assume perfect inner current loop: actual current equals limited command. */
        double t_e = torque_from_dq(i_cmd.i_d, i_cmd.i_q);
        double w_dot = (t_e - tc->load_torque - B_ESTIMATE * w_m) / J_ESTIMATE;
        w_m += w_dot * TS;

        if (fabs(w_m) > fabs(peak_speed)) {
            peak_speed = w_m;
        }
    }

    final_error = tc->w_ref - w_m;

    printf("%s\n", tc->name);
    printf("  ref=% .3f rad/s | load=% .3f Nm | final=% .3f rad/s | error=% .3f rad/s | peak=% .3f rad/s\n",
           tc->w_ref, tc->load_torque, w_m, final_error, peak_speed);

    if (w_m < tc->final_speed_min || w_m > tc->final_speed_max) {
        printf("  FAIL: final speed outside expected range.\n");
        return 1;
    }

    printf("  PASS\n");
    return 0;
}

int main(void)
{
    const speed_case_t cases[] = {
        {
            "Case 1: positive speed step",
            20.0,
            0.0,
            0.0,
            20.0 - TRACKING_TOL_RAD_PER_SEC,
            20.0 + TRACKING_TOL_RAD_PER_SEC
        },
        {
            "Case 2: negative speed step",
            -15.0,
            0.0,
            0.0,
            -15.0 - TRACKING_TOL_RAD_PER_SEC,
            -15.0 + TRACKING_TOL_RAD_PER_SEC
        },
        {
            "Case 3: positive step with load torque",
            18.0,
            0.12,
            0.0,
            18.0 - TRACKING_TOL_RAD_PER_SEC,
            18.0 + TRACKING_TOL_RAD_PER_SEC
        }
    };
    int failures = 0;
    int ncases = (int)(sizeof(cases) / sizeof(cases[0]));

    printf("Testing speed loop with Ts=%g s, J=%g, B=%g, current limit=%g A\n",
           TS, J_ESTIMATE, B_ESTIMATE, VECTOR_CURRENT_LIMIT);

    for (int i = 0; i < ncases; ++i) {
        failures += run_case(&cases[i]);
    }

    if (failures != 0) {
        printf("\nSpeed-loop test failed: %d case(s) out of range.\n", failures);
        return EXIT_FAILURE;
    }

    printf("\nAll speed-loop checks passed.\n");
    return EXIT_SUCCESS;
}
