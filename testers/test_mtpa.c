#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define POLE_PAIRS (4.0)
#define PM_FLUX_V_SEC_PER_RAD (0.0383)
#define L_DS_ESTIMATE (0.001)
#define L_QS_ESTIMATE (0.0016)

#define ANGLE_STEPS (200000)
#define TORQUE_TOLERANCE (1e-4)
#define MAG_TOLERANCE (1e-9)

typedef struct {
    double i_d;
    double i_q;
} dq_current_t;

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

    double iq_mtpa_radicand = i_s_ref * i_s_ref - out.i_d * out.i_d;
    out.i_q = (i_s_ref >= 0.0 ? 1.0 : -1.0) * sqrt(fmax(0.0, iq_mtpa_radicand));

    return out;
}

static dq_current_t brute_force_mtpa(double i_s_ref)
{
    dq_current_t best = {0.0, 0.0};
    double best_torque = -DBL_MAX;
    double magnitude = fabs(i_s_ref);
    double iq_sign = (i_s_ref >= 0.0) ? 1.0 : -1.0;

    if (magnitude == 0.0) {
        return best;
    }

    for (int step = 0; step <= ANGLE_STEPS; ++step) {
        double angle = (M_PI * step) / (double)ANGLE_STEPS;
        double i_d = -magnitude * sin(angle);
        double i_q = iq_sign * magnitude * cos(angle);
        double torque = iq_sign * torque_from_dq(i_d, i_q);

        if (torque > best_torque) {
            best_torque = torque;
            best.i_d = i_d;
            best.i_q = i_q;
        }
    }

    return best;
}

static int check_case(double i_s_ref)
{
    dq_current_t mtpa = compute_mtpa(i_s_ref);
    dq_current_t brute = brute_force_mtpa(i_s_ref);

    double mtpa_mag = sqrt(mtpa.i_d * mtpa.i_d + mtpa.i_q * mtpa.i_q);
    double ref_mag = fabs(i_s_ref);
    double mtpa_torque = torque_from_dq(mtpa.i_d, mtpa.i_q);
    double brute_torque = torque_from_dq(brute.i_d, brute.i_q);
    double torque_error = fabs(fabs(mtpa_torque) - fabs(brute_torque));
    double mag_error = fabs(mtpa_mag - ref_mag);
    int iq_sign_ok = (ref_mag == 0.0) || ((i_s_ref > 0.0 && mtpa.i_q >= 0.0) || (i_s_ref < 0.0 && mtpa.i_q <= 0.0));

    printf("i_s_ref=%7.3f | mtpa=(id=% .6f, iq=% .6f) | brute=(id=% .6f, iq=% .6f)\n",
           i_s_ref, mtpa.i_d, mtpa.i_q, brute.i_d, brute.i_q);
    printf("             | |T_mtpa|=% .8f | |T_brute|=% .8f | torque_err=% .8e | mag_err=% .8e\n",
           fabs(mtpa_torque), fabs(brute_torque), torque_error, mag_error);

    if (mag_error > MAG_TOLERANCE) {
        printf("FAIL: current magnitude does not match |i_s_ref|.\n");
        return 1;
    }

    if (!iq_sign_ok) {
        printf("FAIL: iq sign does not match the requested torque direction.\n");
        return 1;
    }

    if (torque_error > TORQUE_TOLERANCE) {
        printf("FAIL: closed-form MTPA does not match brute-force optimum.\n");
        return 1;
    }

    if ((L_QS_ESTIMATE > L_DS_ESTIMATE) && (fabs(i_s_ref) > 0.0) && (mtpa.i_d > 1e-9)) {
        printf("FAIL: expected negative or zero d-axis current for IPM MTPA.\n");
        return 1;
    }

    printf("PASS\n");
    return 0;
}

int main(void)
{
    const double test_cases[] = {
        -8.0, -6.0, -4.0, -2.0, -1.0, -0.25,
         0.0,
         0.25, 1.0, 2.0, 4.0, 6.0, 8.0
    };
    int failures = 0;
    int num_cases = (int)(sizeof(test_cases) / sizeof(test_cases[0]));

    printf("Testing MTPA implementation with Ld=%g H, Lq=%g H, lambda_pm=%g V/(rad/s)\n",
           L_DS_ESTIMATE, L_QS_ESTIMATE, PM_FLUX_V_SEC_PER_RAD);

    for (int i = 0; i < num_cases; ++i) {
        failures += check_case(test_cases[i]);
    }

    if (failures != 0) {
        printf("\nMTPA test failed: %d case(s) did not match the expected optimum.\n", failures);
        return EXIT_FAILURE;
    }

    printf("\nAll MTPA checks passed.\n");
    return EXIT_SUCCESS;
}
