
#ifdef APP_WOLFPACK

#include "task_wolfpack.h"
#include "drv/amds.h"
#include "drv/analog.h"
#include "drv/cpu_timer.h"
#include "drv/gp3io_mux.h"
#include "drv/pwm.h"
#include "drv/timing_manager.h"
#include "sys/scheduler.h"
#include "sys/task_stats.h"
#include "drv/encoder.h"
#include <math.h>
#include "sys/transform.h"
#include "stdlib.h"
#include "xil_printf.h"

// ============================================================================
// ENCODER PARAMETERS
// ============================================================================

#define ENCODER_COUNTS       (20000.0)              // Encoder counts per mechanical revolution
#define ENCODER_COUNTS_INV   (1.0 * (1/ENCODER_COUNTS)) // Inverse of encoder counts per rev
#define THETA_M_OFFSET       (1.0 * (0))            // Offset added to raw encoder angle [rad]
#define ENCODER_OFFSET       (PIOVER2)              // AMDS encoder1 offset [rad/count]

// ============================================================================
// SENSOR SCALING PARAMETERS
// ============================================================================

#define HV_SENSE_GAIN              (0.0163)         // AMDS HV Sense gain, ch1 [V/count]
#define HV_SENSE_OFFSET_COUNTS     (32768)          // HV Sense nominal offset [counts]
#define CURRENT_SENSE_GAIN         (0.002028)       // Current sensor gain [A/count]
#define CURRENT_SENSE_OFFSET_COUNTS (36572)         // Current sensor nominal offset [counts]

// ============================================================================
// CALIBRATION AND FILTERING
// ============================================================================

#define CALIB_SAMPLES   (10000)                     // Samples for sensor offset calibration
#define EXP_W_F_TS      (1.0 * (0.9937))           // LPF coeff = exp(-2*pi*fc*Ts), fc=10 Hz
#define ONE_OVER_PI2    (0.15915494309)             // 1 / (2*pi)

// ============================================================================
// MOTOR PARAMETERS
// ============================================================================

#define POLE_PAIRS          (4.0)                   // Number of pole pairs
#define POLE_PAIRS_INV      (1 / POLE_PAIRS)        // Inverse of pole pairs
#define PM_FLUX_V_SEC_PER_RAD (0.0383)              // PM flux constant [V/(elec rad/s)]
#define L_DS_ESTIMATE       (0.001)                 // d-axis stator inductance estimate [H]
#define L_QS_ESTIMATE       (0.0016)                // q-axis stator inductance estimate [H]
#define R_S_ESTIMATE        (0.55)                  // Stator resistance estimate [Ohms]

// ============================================================================
// PROTECTION LIMITS
// ============================================================================

#define WOLFPACK_VOLTAGE_MAX    (70)                // Over-voltage trip level [V]
#define WOLFPACK_CURRENT_MAX    (12)                // Over-current trip level [A]
#define VECTOR_CURRENT_LIMIT    (8)                 // Current vector command limit [A] (8.0 for normal op)

// ============================================================================
// CURRENT REGULATOR (IREG) PARAMETERS
// ============================================================================

#define IREG_W_GCF          (1885)                  // Current regulator gain crossover freq [rad/s]
#define IREG_W_PI_CROSS_OVER (500)                  // Current PI crossover frequency [rad/s]
#define IREG_KPD            (0.001  * IREG_W_GCF)  // d-axis proportional gain
#define IREG_KPQ            (0.0016 * IREG_W_GCF)  // q-axis proportional gain
#define IREG_KID            (IREG_W_PI_CROSS_OVER * IREG_KPD)  // d-axis integral gain
#define IREG_KIQ            (IREG_W_PI_CROSS_OVER * IREG_KPQ)  // q-axis integral gain

// ============================================================================
// SPEED REGULATOR PARAMETERS (Lab 4-1)
// Tune J_ESTIMATE and B_ESTIMATE from Test 1.1.
// Adjust SPEED_REG_W_GCF for bandwidth trials: 2*pi*1, 2*pi*3, 2*pi*10.
// ============================================================================

#define J_ESTIMATE          (0.0042668)             // Rotational inertia estimate [kg*m^2]
#define B_ESTIMATE          (0.0020483)             // Rotational damping estimate [N*m*s/rad]
#define SPEED_REG_W_GCF     (6.28318530718)         // Speed regulator GCF [rad/s] (default: 2*pi*1)
#define Kp_w_m              (J_ESTIMATE * SPEED_REG_W_GCF)  // Speed PI proportional gain [N*m*s/rad]
#define Ki_w_m              (B_ESTIMATE * SPEED_REG_W_GCF)  // Speed PI integral gain [N*m/rad]

// ============================================================================
// FIELD WEAKENING REGULATOR PARAMETERS (Lab 4-2)
// FW_REG_KI is tuned so the FW loop bandwidth matches the speed regulator GCF.
// K_iFW = w_GCF_w / (w_e_nom * L_d)
// ============================================================================

#define W_E_NOM_FW       (500.0)                    // Nominal elec. speed for KI tuning [rad/s]
#define FW_REG_KI        (SPEED_REG_W_GCF / (W_E_NOM_FW * L_DS_ESTIMATE)) // FW integrator gain [1/H]
#define FW_DC_TO_AC_GAIN (0.95 / 1.73205080757)     // k/sqrt(3): DC-to-AC voltage headroom gain

// ============================================================================
// SCHEDULER AND TIMING
// ============================================================================

const uint8_t amds_port = 1;
int    LOG_amds_valid        = 0;       // AMDS data validity flag
double Ts                    = 1.0 / (double)TASK_WOLFPACK_UPDATES_PER_SEC; // Sample period [s]

double LOG_control_looptime  = 0;       // ISR period [us]
double LOG_control_runtime   = 0;       // ISR execution time [us]
static uint32_t last_now_start = 0;
static task_control_block_t tcb;        // Scheduler task control block

// ============================================================================
// FEATURE ENABLE FLAGS  (1 = enabled, 0 = bypassed)
// Toggle at runtime via task_wolfpack_set_en_*() setters.
// ============================================================================

int en_speed_loop   = 1;   // Speed PI regulator (0 → manual iq/id via ref_manual)
int en_mtpa         = 1;   // MTPA optimal angle  (0 → id_mtpa=0, iq_mtpa=is_ref)
int en_fw           = 1;   // Field weakening integrator
int en_state_fb     = 0;   // State-feedback decoupling (0 → classical PI + q-BEMF)
int en_current_loop = 1;   // Current PI regulator (0 → v_cmd zeroed, integrals cleared)
int en_vector_limit = 1;   // Vector current limiter (0 → refs passed through unclipped)

// ============================================================================
// STATE MACHINE
// ============================================================================

int LOG_wolf_state       = 1;   // Present state: 0=CALIBRATE, 1=IDLE, 2=RUNNING, 3=TRIPPED
int wolf_state_prev      = 1;   // Previous state (for entry-action detection)
int sm_request_idle      = 0;   // Request transition to IDLE
int sm_request_run       = 0;   // Request transition to RUNNING
int sm_request_calibrate = 0;   // Request transition to CALIBRATE
int sm_request_trip_clear= 0;   // Request trip clear from TRIPPED

int calibrate_status     = 0;   // 1 = calibration complete, 0 = in progress
int calibrate_count      = 0;   // Current calibration sample count

int LOG_pwm_state        = 0;   // PWM enable status: 0 = disabled, 1 = enabled

// ============================================================================
// PROTECTION STATUS
// ============================================================================

int LOG_OV_status_dc      = 0;  // DC over-voltage flag
int LOG_OC_status_a       = 0;  // Phase A over-current flag
int LOG_OC_status_b       = 0;  // Phase B over-current flag
int LOG_OC_status_c       = 0;  // Phase C over-current flag
int LOG_protection_status = 0;  // Combined protection trip flag

// ============================================================================
// SENSOR RAW COUNTS (AMDS)
// ============================================================================

uint32_t LOG_enc_pos_data    = 0;   // Encoder position [counts]
uint32_t LOG_amds_ch_1_data  = 0;   // DC bus voltage [counts]
uint32_t LOG_amds_ch_2_data  = 0;   // Phase A current [counts]
uint32_t LOG_amds_ch_4_data  = 0;   // Phase B current [counts]
uint32_t LOG_amds_ch_6_data  = 0;   // DC bus current [counts]

// ============================================================================
// SCALED SENSOR VALUES AND OFFSETS
// ============================================================================

double LOG_v_dc         = 0;    // DC bus voltage [V]
double LOG_v_dc_offset  = 0;    // DC bus voltage sensor offset [V]

double LOG_i_a          = 0;    // Phase A current [A]
double LOG_i_b          = 0;    // Phase B current [A]
double LOG_i_c          = 0;    // Phase C current (reconstructed) [A]
double LOG_i_a_offset   = 0;    // Phase A current sensor offset [A]
double LOG_i_b_offset   = 0;    // Phase B current sensor offset [A]

double LOG_i_dc         = 0;    // DC bus current [A]
double LOG_i_dc_offset  = 0;    // DC bus current sensor offset [A]

// ============================================================================
// ROTOR ANGLE AND SPEED
// ============================================================================

double LOG_theta_m          = 0;            // Mechanical rotor angle [rad]
double LOG_theta_e          = 0;            // Electrical rotor angle [rad]
double theta_e_offset       = -1.047197;    // Park frame angle offset [rad]
double LOG_theta_e_ref_frame= 0;            // Angle used in Park transforms [rad]

double theta_m_prev         = 0;            // Previous mechanical angle [rad]
double LOG_delta_theta_m    = 0;            // Incremental mechanical angle [rad]

double LOG_w_m              = 0;            // Mechanical speed [rad/s]
double LOG_w_m_RPM          = 0;            // Mechanical speed [RPM]
double LOG_w_m_RPM_filtered = 0;            // Filtered mechanical speed [RPM]
double LOG_w_m_filtered     = 0;            // Filtered mechanical speed [rad/s]
double LOG_w_e_filtered     = 0;            // Filtered electrical speed [rad/s]

// ============================================================================
// TORQUE ESTIMATE
// ============================================================================

double LOG_torque_estimate  = 0;            // Electromagnetic torque estimate [N*m]

// ============================================================================
// SPEED PI REGULATOR (Lab 4-1)
// ============================================================================

double LOG_w_m_ref          = 0;    // Speed reference [rad/s]
double w_m_error            = 0;    // Speed error: w_ref - w_filtered [rad/s]
double LOG_T_e_cmd_prop     = 0;    // Speed PI proportional torque command [N*m]
double LOG_T_e_cmd_inte     = 0;    // Speed PI integral torque command [N*m]
double LOG_T_e_cmd          = 0;    // Total speed PI torque command [N*m]

// ============================================================================
// MTPA CURRENT REFERENCES
// ============================================================================

double LOG_i_s_ref          = 0;    // MTPA stator current magnitude reference [A]
double LOG_i_d_mtpa         = 0;    // MTPA d-axis current reference [A]
double LOG_i_q_mtpa         = 0;    // MTPA q-axis current reference [A]

// ============================================================================
// FIELD WEAKENING REGULATOR (Lab 4-2)
// ============================================================================

double LOG_v_dq_mag_cmd         = 0;    // Voltage magnitude command (FW reference) [V]
double LOG_v_dq_mag             = 0;    // Actual voltage magnitude from previous ISR [V]
double LOG_v_dq_mag_error_inte  = 0;    // FW integrator state [V*s]
double LOG_i_d_fw               = 0;    // FW d-axis current output (always <= 0) [A]

// ============================================================================
// COMBINED CURRENT REFERENCES AND ERRORS
// ============================================================================

double LOG_i_d_ref_manual   = 0;    // Manual d-axis current reference [A]
double LOG_i_q_ref_manual   = 0;    // Manual q-axis current reference [A]

double LOG_i_d_ref          = 0;    // Total d-axis reference (MTPA + FW + manual) [A]
double LOG_i_q_ref          = 0;    // Total q-axis reference (MTPA + manual) [A]

double LOG_i_d_ref_limited  = 0;    // d-axis reference after vector limiter [A]
double LOG_i_q_ref_limited  = 0;    // q-axis reference after vector limiter [A]

double LOG_i_d              = 0;    // d-axis current feedback [A]
double LOG_i_q              = 0;    // q-axis current feedback [A]
double LOG_i_0              = 0;    // Zero-sequence current [A]

double LOG_i_d_Error        = 0;    // d-axis current error (ref - feedback) [A]
double LOG_i_q_Error        = 0;    // q-axis current error (ref - feedback) [A]
double LOG_i_d_Error_Integral = 0;  // d-axis current error integral [A*s]
double LOG_i_q_Error_Integral = 0;  // q-axis current error integral [A*s]

// ============================================================================
// CURRENT REGULATOR VOLTAGE COMMANDS (dq)
// ============================================================================

double LOG_v_cmd_d_BEMF     = 0;    // d-axis feedforward / decoupling term [V]
double LOG_v_cmd_d_Prop     = 0;    // d-axis PI proportional term [V]
double LOG_v_cmd_d_Inte     = 0;    // d-axis PI integral term [V]
double LOG_v_cmd_d          = 0;    // Total d-axis voltage command [V]

double LOG_v_cmd_q_BEMF     = 0;    // q-axis feedforward / BEMF term [V]
double LOG_v_cmd_q_Prop     = 0;    // q-axis PI proportional term [V]
double LOG_v_cmd_q_Inte     = 0;    // q-axis PI integral term [V]
double LOG_v_cmd_q          = 0;    // Total q-axis voltage command [V]

double LOG_v_cmd_0          = 0;    // Zero-sequence voltage command [V]

// ============================================================================
// POWER ESTIMATES (Lab 4-2)
// ============================================================================

double LOG_p_dc     = 0;    // DC input power: Vdc * Idc [W]
double LOG_p_ac     = 0;    // AC stator power: (3/2)(Vcmd_q*Iq + Vcmd_d*Id) [W]
double LOG_p_mech   = 0;    // Mechanical shaft power: Te * wm_filtered [W]

// ============================================================================
// INVERTER OUTPUT: ABC VOLTAGE, MODULATION, AND DUTY COMMANDS
// ============================================================================

double i_abc[3]     = {0, 0, 0};    // Phase current vector [A]
double i_dq0[3]     = {0, 0, 0};    // dq0 current vector [A]
double v_cmd_abc[3] = {0, 0, 0};    // Phase voltage command vector [V]
double v_cmd_dq0[3] = {0, 0, 0};    // dq0 voltage command vector [V]

double LOG_v_cmd_a  = 0;            // Phase A voltage command [V]
double LOG_v_cmd_b  = 0;            // Phase B voltage command [V]
double LOG_v_cmd_c  = 0;            // Phase C voltage command [V]
double LOG_v_cmd_ab = 0;            // Line-to-line AB voltage command [V]

double LOG_m_cmd_a  = 0;            // Phase A modulation command
double LOG_m_cmd_b  = 0;            // Phase B modulation command
double LOG_m_cmd_c  = 0;            // Phase C modulation command
double LOG_m_cmd_ab = 0;            // Line-to-line AB modulation command
double LOG_m_cmd_0  = 0;            // SVPWM common-mode injection term

double LOG_duty_a   = 0.5;          // Phase A duty ratio [0, 1]
double LOG_duty_b   = 0.5;          // Phase B duty ratio [0, 1]
double LOG_duty_c   = 0.5;          // Phase C duty ratio [0, 1]

// ============================================================================
// TASK INIT / DEINIT
// ============================================================================

int task_wolfpack_init(void)
{
	if (scheduler_tcb_is_registered(&tcb)) {
		return FAILURE;
	}
	scheduler_tcb_init(&tcb, task_wolfpack_callback, NULL, "wolfpack", TASK_WOLFPACK_INTERVAL_USEC);
	//task_stats_enable(&tcb.stats);
	return scheduler_tcb_register(&tcb);
	// return scheduler_tcb_register_high_priority(&tcb);
}

int task_wolfpack_deinit(void)
{
	pwm_disable();
	return scheduler_tcb_unregister(&tcb);
}

// ============================================================================
// ISR CALLBACK  (runs every Ts = 1/10000 s)
// Flow: Sample → Angle/Speed → Scale → Protect → State Machine →
//       Clarke/Park → Torque → Speed PI → MTPA → FW → Refs →
//       Vector Limit → Current Reg → Power → Inv. Park → SVPWM → PWM out
// ============================================================================

void task_wolfpack_callback(void *arg)
{
	// --- Timing: measure loop period ---
	uint32_t now_start = cpu_timer_now();
	uint32_t looptime  = now_start - last_now_start;
	last_now_start     = now_start;
	LOG_control_looptime = cpu_timer_ticks_to_usec(looptime);

	// --- Sample sensors ---
	encoder_get_position(&LOG_enc_pos_data);
	amds_get_data(1, AMDS_CH_1, &LOG_amds_ch_1_data);	// DC bus voltage
	amds_get_data(1, AMDS_CH_2, &LOG_amds_ch_2_data);	// Phase A current
	amds_get_data(1, AMDS_CH_4, &LOG_amds_ch_4_data);	// Phase B current
	amds_get_data(1, AMDS_CH_6, &LOG_amds_ch_6_data);	// DC bus current
	LOG_amds_valid = amds_check_data_validity(1);

	// --- Encoder: counts to mechanical angle [0, 2*pi] ---
	LOG_theta_m = (double)PI2 * (LOG_enc_pos_data) * (double)ENCODER_COUNTS_INV + (double)THETA_M_OFFSET;
	if (LOG_theta_m < 0) {
		LOG_theta_m += PI2 * fabs(floor(LOG_theta_m * ONE_OVER_PI2));
	}
	LOG_theta_m = fmod(LOG_theta_m, PI2);
	LOG_theta_e = fmod(LOG_theta_m * POLE_PAIRS + PI2, PI2);

	// --- Speed: delta angle → rad/s, filter, convert to RPM and elec ---
	LOG_delta_theta_m = LOG_theta_m - theta_m_prev;
	if      (LOG_delta_theta_m < -PI) { LOG_delta_theta_m = LOG_theta_m - theta_m_prev + PI2; }
	else if (LOG_delta_theta_m >  PI) { LOG_delta_theta_m = LOG_theta_m - theta_m_prev - PI2; }

	LOG_w_m             = LOG_delta_theta_m * TASK_WOLFPACK_UPDATES_PER_SEC;
	LOG_w_m_RPM         = RAD_PER_SEC_TO_RPM(LOG_w_m);
	LOG_w_m_RPM_filtered= (1 - EXP_W_F_TS)*LOG_w_m_RPM + LOG_w_m_RPM_filtered*EXP_W_F_TS;
	LOG_w_m_filtered    = RPM_TO_RAD_PER_SEC(LOG_w_m_RPM_filtered);
	LOG_w_e_filtered    = POLE_PAIRS * LOG_w_m_filtered;

	// --- Scale raw counts to physical units (subtract calibrated offsets) ---
	if (LOG_wolf_state != 0) {  // Normal operation: apply offsets
		LOG_v_dc  = HV_SENSE_GAIN     * ((LOG_amds_ch_1_data*1.0) - HV_SENSE_OFFSET_COUNTS)      - LOG_v_dc_offset;
		LOG_i_a   = CURRENT_SENSE_GAIN * ((LOG_amds_ch_2_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS) - LOG_i_a_offset;
		LOG_i_b   = CURRENT_SENSE_GAIN * ((LOG_amds_ch_4_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS) - LOG_i_b_offset;
		LOG_i_c   = -(LOG_i_a + LOG_i_b);
		LOG_i_dc  = CURRENT_SENSE_GAIN * ((LOG_amds_ch_6_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS) - LOG_i_dc_offset;
	} else {                    // Calibrate state: do NOT subtract offset (would corrupt it)
		LOG_v_dc  = HV_SENSE_GAIN     * ((LOG_amds_ch_1_data*1.0) - HV_SENSE_OFFSET_COUNTS);
		LOG_i_a   = CURRENT_SENSE_GAIN * ((LOG_amds_ch_2_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS);
		LOG_i_b   = CURRENT_SENSE_GAIN * ((LOG_amds_ch_4_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS);
		LOG_i_c   = -(LOG_i_a + LOG_i_b);
		LOG_i_dc  = CURRENT_SENSE_GAIN * ((LOG_amds_ch_6_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS);
	}

	// --- Protection: check OV / OC limits ---
	LOG_OV_status_dc     = (fabs(LOG_v_dc) > WOLFPACK_VOLTAGE_MAX);
	LOG_OC_status_a      = (fabs(LOG_i_a)  > WOLFPACK_CURRENT_MAX);
	LOG_OC_status_b      = (fabs(LOG_i_b)  > WOLFPACK_CURRENT_MAX);
	LOG_OC_status_c      = (fabs(LOG_i_c)  > WOLFPACK_CURRENT_MAX);
	LOG_protection_status = (LOG_OC_status_a || LOG_OC_status_b || LOG_OC_status_c || LOG_OV_status_dc);

	// ========================================================================
	// STATE MACHINE
	// ========================================================================
	switch (LOG_wolf_state)
	{
	case 0: // CALIBRATE — PWM off, accumulate sensor offsets via LPF
		LOG_pwm_state = pwm_disable();
		if (!calibrate_status) {
			calibrate_count++;
		}
		LOG_v_dc_offset  = (1.0 - EXP_W_F_TS)*LOG_v_dc  + LOG_v_dc_offset *EXP_W_F_TS;
		LOG_i_a_offset   = (1.0 - EXP_W_F_TS)*LOG_i_a   + LOG_i_a_offset  *EXP_W_F_TS;
		LOG_i_b_offset   = (1.0 - EXP_W_F_TS)*LOG_i_b   + LOG_i_b_offset  *EXP_W_F_TS;
		LOG_i_dc_offset  = (1.0 - EXP_W_F_TS)*LOG_i_dc  + LOG_i_dc_offset *EXP_W_F_TS;
		if (calibrate_count >= CALIB_SAMPLES) {
			calibrate_status = 1;
			LOG_wolf_state   = 1;   // -> IDLE
		}
		break;

	case 1: // IDLE — PWM off, all integrators and commands cleared
		LOG_pwm_state = pwm_disable();
		LOG_i_q_Error_Integral   = 0;
		LOG_i_d_Error_Integral   = 0;
		LOG_i_d_ref_manual       = 0;
		LOG_i_q_ref_manual       = 0;
		LOG_w_m_ref              = 0;
		LOG_T_e_cmd_prop         = 0;
		LOG_T_e_cmd_inte         = 0;
		LOG_T_e_cmd              = 0;
		LOG_v_dq_mag_error_inte  = 0;
		LOG_i_d_fw               = 0;
		if (LOG_protection_status) {
			LOG_wolf_state = 3;         // -> TRIPPED
		} else if (sm_request_calibrate == 1) {
			LOG_wolf_state   = 0;       // -> CALIBRATE
			calibrate_status = 0;
			calibrate_count  = 0;
		} else if (sm_request_run == 1) {
			LOG_wolf_state = 2;         // -> RUNNING
		}
		break;

	case 2: // RUNNING — PWM enabled
		if (LOG_wolf_state != wolf_state_prev) {
			// Entry actions (none currently)
		}
		if (LOG_protection_status) {
			LOG_wolf_state = 3;         // -> TRIPPED
		} else if (sm_request_idle == 1) {
			LOG_wolf_state = 1;         // -> IDLE
		}
		LOG_pwm_state = pwm_enable();
		break;

	default: // TRIPPED — PWM off, wait for clear request
		LOG_pwm_state = pwm_disable();
		if (LOG_protection_status == 0 && sm_request_trip_clear == 1) {
			sm_request_trip_clear = 0;
			LOG_wolf_state        = 1;  // -> IDLE
		}
		break;
	}

	// Clear all one-shot state-transition requests
	sm_request_trip_clear = 0;
	sm_request_run        = 0;
	sm_request_idle       = 0;
	sm_request_calibrate  = 0;

	// ========================================================================
	// CLARKE + PARK TRANSFORM  (abc → dq0)
	// ========================================================================

	i_abc[0] = LOG_i_a;
	i_abc[1] = LOG_i_b;
	i_abc[2] = LOG_i_c;

	LOG_theta_e_ref_frame = LOG_theta_e + theta_e_offset;
	LOG_theta_e           = fmod(LOG_theta_e_ref_frame + PI2, PI2);

	transform_dqz(0, LOG_theta_e_ref_frame, i_abc, i_dq0);

	LOG_i_d = i_dq0[0];
	LOG_i_q = i_dq0[1];
	LOG_i_0 = i_dq0[2];

	// ========================================================================
	// TORQUE ESTIMATE
	// T_e = (3/2) * P * [lambda_pm * iq + iq * id * (Ld - Lq)]
	// ========================================================================

	LOG_torque_estimate = 1.5 * POLE_PAIRS
	    * (PM_FLUX_V_SEC_PER_RAD * LOG_i_q
	       + LOG_i_q * LOG_i_d * (L_DS_ESTIMATE - L_QS_ESTIMATE));

	// ========================================================================
	// SPEED PI REGULATOR (Lab 4-1)
	// Kp = J*w_GCF,  Ki = B*w_GCF
	// ========================================================================

	w_m_error = LOG_w_m_ref - LOG_w_m_filtered;
	if (en_speed_loop) {
		LOG_T_e_cmd_prop  = Kp_w_m * w_m_error;
		LOG_T_e_cmd_inte += Ki_w_m * w_m_error * Ts;
		LOG_T_e_cmd       = LOG_T_e_cmd_prop + LOG_T_e_cmd_inte;
	} else {
		LOG_T_e_cmd_prop = 0;
		LOG_T_e_cmd_inte = 0;
		LOG_T_e_cmd      = 0;
	}

	// ========================================================================
	// MTPA: torque command → is_ref → (id_mtpa, iq_mtpa)
	// id_mtpa = (lpm - sqrt(lpm^2 + 8*(Lq-Ld)^2*is^2)) / (4*(Lq-Ld))
	// iq_mtpa = sign(is) * sqrt(is^2 - id_mtpa^2)
	// ========================================================================

	LOG_i_s_ref = LOG_T_e_cmd / (1.5 * POLE_PAIRS * PM_FLUX_V_SEC_PER_RAD);

	if (en_mtpa) {
		double dL = L_QS_ESTIMATE - L_DS_ESTIMATE;
		double mtpa_radicand = PM_FLUX_V_SEC_PER_RAD * PM_FLUX_V_SEC_PER_RAD
		    + 8.0 * dL * dL * LOG_i_s_ref * LOG_i_s_ref;
		LOG_i_d_mtpa = (PM_FLUX_V_SEC_PER_RAD - sqrt(mtpa_radicand))
		    / (4.0 * dL);
		double iq_mtpa_radicand = LOG_i_s_ref * LOG_i_s_ref - LOG_i_d_mtpa * LOG_i_d_mtpa;
		LOG_i_q_mtpa = (LOG_i_s_ref >= 0.0 ? 1.0 : -1.0) * sqrt(fmax(0.0, iq_mtpa_radicand));
	} else {
		// SPM assumption: id=0, all current on q-axis
		LOG_i_d_mtpa = 0.0;
		LOG_i_q_mtpa = LOG_i_s_ref;
	}

	// ========================================================================
	// FIELD WEAKENING REGULATOR (Lab 4-2)
	// Integrates (Vdq_mag_cmd - Vdq_mag); output id_fw <= 0.
	// Uses v_cmd_d / v_cmd_q from the PREVIOUS ISR (one-sample delay).
	// ========================================================================

	LOG_v_dq_mag     = sqrt(LOG_v_cmd_d * LOG_v_cmd_d + LOG_v_cmd_q * LOG_v_cmd_q);
	LOG_v_dq_mag_cmd = FW_DC_TO_AC_GAIN * LOG_v_dc;

	if (en_fw && LOG_wolf_state == 2) {
		double fw_error = LOG_v_dq_mag_cmd - LOG_v_dq_mag; // x[k], allowed + or -
		LOG_v_dq_mag_error_inte += Ts * fw_error;           // y[k] = y[k-1] + Ts*x[k]
		if (LOG_v_dq_mag_error_inte > 0.0) {
			LOG_v_dq_mag_error_inte = 0.0;                  // Clamp integrator <= 0
		}
	} else if (!en_fw) {
		LOG_v_dq_mag_error_inte = 0.0;                      // Flush when disabled
	}
	LOG_i_d_fw = FW_REG_KI * LOG_v_dq_mag_error_inte;      // id_fw <= 0

	// ========================================================================
	// COMBINE CURRENT REFERENCES
	// id_ref = min(id_mtpa, id_fw)  — FW pulls id more negative when active
	// ========================================================================

	LOG_i_d_ref = fmin(LOG_i_d_mtpa, LOG_i_d_fw) + LOG_i_d_ref_manual;
	LOG_i_q_ref = LOG_i_q_mtpa + LOG_i_q_ref_manual;

	// ========================================================================
	// VECTOR LIMITER WITH d-AXIS PRIORITY (Lab 4-2)
	// id is clamped first; remaining current budget is given to iq.
	// ========================================================================

	if (en_vector_limit) {
		double id_clamped = fmax(-VECTOR_CURRENT_LIMIT, fmin(VECTOR_CURRENT_LIMIT, LOG_i_d_ref));
		double iq_budget  = sqrt(fmax(0.0, VECTOR_CURRENT_LIMIT * VECTOR_CURRENT_LIMIT
		                               - id_clamped * id_clamped));
		LOG_i_d_ref_limited = id_clamped;
		LOG_i_q_ref_limited = (LOG_i_q_ref >= 0.0 ? 1.0 : -1.0) * fmin(fabs(LOG_i_q_ref), iq_budget);
	} else {
		LOG_i_d_ref_limited = LOG_i_d_ref;
		LOG_i_q_ref_limited = LOG_i_q_ref;
	}

	// ========================================================================
	// CURRENT REGULATOR (PI)
	// ========================================================================

	if (en_current_loop) {
		LOG_i_d_Error = LOG_i_d_ref_limited - LOG_i_d;
		LOG_i_q_Error = LOG_i_q_ref_limited - LOG_i_q;

		LOG_i_d_Error_Integral += LOG_i_d_Error * Ts;
		LOG_i_q_Error_Integral += LOG_i_q_Error * Ts;

		// Feedforward / decoupling terms
		if (en_state_fb) {
			// State Feedback Decoupling — cancel cross-coupling + BEMF
			//   vd += +we*Lq*iq   (cancel d-axis coupling)
			//   vq += -we*lpm + we*Ld*id  (BEMF + cancel q-axis coupling)
			LOG_v_cmd_d_BEMF = +LOG_w_e_filtered * L_QS_ESTIMATE * LOG_i_q;
			LOG_v_cmd_q_BEMF = -PM_FLUX_V_SEC_PER_RAD * LOG_w_e_filtered
			                   + LOG_w_e_filtered * L_DS_ESTIMATE * LOG_i_d;
		} else {
			// Classical PI — BEMF feedforward on q-axis only
			LOG_v_cmd_d_BEMF = 0;
			LOG_v_cmd_q_BEMF = PM_FLUX_V_SEC_PER_RAD * LOG_w_e_filtered;
		}

		LOG_v_cmd_d_Prop = IREG_KPD * LOG_i_d_Error;
		LOG_v_cmd_d_Inte = IREG_KID * LOG_i_d_Error_Integral;
		LOG_v_cmd_d      = LOG_v_cmd_d_BEMF + LOG_v_cmd_d_Prop + LOG_v_cmd_d_Inte;

		LOG_v_cmd_q_Prop = IREG_KPQ * LOG_i_q_Error;
		LOG_v_cmd_q_Inte = IREG_KIQ * LOG_i_q_Error_Integral;
		LOG_v_cmd_q      = LOG_v_cmd_q_BEMF + LOG_v_cmd_q_Prop + LOG_v_cmd_q_Inte;
	} else {
		// Current loop bypassed — zero all outputs and flush integrators
		LOG_i_d_Error          = 0; LOG_i_q_Error          = 0;
		LOG_i_d_Error_Integral = 0; LOG_i_q_Error_Integral = 0;
		LOG_v_cmd_d_BEMF = 0; LOG_v_cmd_q_BEMF = 0;
		LOG_v_cmd_d_Prop = 0; LOG_v_cmd_q_Prop = 0;
		LOG_v_cmd_d_Inte = 0; LOG_v_cmd_q_Inte = 0;
		LOG_v_cmd_d      = 0; LOG_v_cmd_q      = 0;
	}

	// ========================================================================
	// POWER ESTIMATES (Lab 4-2)
	// ========================================================================

	LOG_p_dc   = LOG_v_dc * LOG_i_dc;
	LOG_p_ac   = 1.5 * (LOG_v_cmd_q * LOG_i_q + LOG_v_cmd_d * LOG_i_d);
	LOG_p_mech = LOG_torque_estimate * LOG_w_m_filtered;

	// ========================================================================
	// INVERSE PARK TRANSFORM  (dq0 → abc)
	// ========================================================================

	v_cmd_dq0[0] = LOG_v_cmd_d;
	v_cmd_dq0[1] = LOG_v_cmd_q;
	v_cmd_dq0[2] = LOG_v_cmd_0;

	transform_dqz_inverse(0, LOG_theta_e_ref_frame, v_cmd_abc, v_cmd_dq0);

	LOG_v_cmd_a  = v_cmd_abc[0];
	LOG_v_cmd_b  = v_cmd_abc[1];
	LOG_v_cmd_c  = v_cmd_abc[2];
	LOG_v_cmd_ab = LOG_v_cmd_a - LOG_v_cmd_b;

	// ========================================================================
	// SVPWM MODULATION
	// ========================================================================

	// Scale by 2/Vdc for per-unit modulation index
	LOG_m_cmd_a = 2 * LOG_v_cmd_a / LOG_v_dc;
	LOG_m_cmd_b = 2 * LOG_v_cmd_b / LOG_v_dc;
	LOG_m_cmd_c = 2 * LOG_v_cmd_c / LOG_v_dc;

	// SVPWM: inject common-mode min-max average
	LOG_m_cmd_0 = -0.5 * (fmax(LOG_m_cmd_a, fmax(LOG_m_cmd_b, LOG_m_cmd_c))
	                     + fmin(LOG_m_cmd_a, fmin(LOG_m_cmd_b, LOG_m_cmd_c)));
	LOG_m_cmd_a  += LOG_m_cmd_0;
	LOG_m_cmd_b  += LOG_m_cmd_0;
	LOG_m_cmd_c  += LOG_m_cmd_0;
	LOG_m_cmd_ab  = LOG_m_cmd_a - LOG_m_cmd_b;

	// Level-shift to duty ratio [0, 1]
	LOG_duty_a = 0.5 * (1 + LOG_m_cmd_a);
	LOG_duty_b = 0.5 * (1 + LOG_m_cmd_b);
	LOG_duty_c = 0.5 * (1 + LOG_m_cmd_c);

	// Write to PWM peripheral
	pwm_set_duty(0, LOG_duty_a);    // HB1 (PWM1/2)
	pwm_set_duty(1, LOG_duty_b);    // HB2 (PWM3/4)
	pwm_set_duty(2, LOG_duty_c);    // HB3 (PWM5/6)

	// ========================================================================
	// END OF ISR — bookkeeping for next cycle
	// ========================================================================

	theta_m_prev   = LOG_theta_m;
	wolf_state_prev = LOG_wolf_state;

	uint32_t now_end = cpu_timer_now();
	uint32_t runtime = now_end - now_start;
	LOG_control_runtime = cpu_timer_ticks_to_usec(runtime);
}

// ============================================================================
// STATE MACHINE COMMAND INTERFACE
// ============================================================================

void task_wolfpack_sm_run(void)        { sm_request_run = 1; }
void task_wolfpack_sm_idle(void)       { sm_request_idle = 1; }
void task_wolfpack_sm_trip_clear(void) { sm_request_trip_clear = 1; }

void task_wolfpack_sm_calibrate(void)
{
	sm_request_calibrate = 1;
	calibrate_status     = 0;
}

int task_wolfpack_sm_get_state(void)
{
	return LOG_wolf_state;
}

// ============================================================================
// SETPOINT SETTERS
// ============================================================================

int task_wolfpack_set_i_q_ref_manual(double i) { LOG_i_q_ref_manual = i; return SUCCESS; }
int task_wolfpack_set_i_d_ref_manual(double i) { LOG_i_d_ref_manual = i; return SUCCESS; }
int task_wolfpack_set_w_m_ref(double w)        { LOG_w_m_ref = w;        return SUCCESS; }

// ============================================================================
// FEATURE FLAG SETTERS
// ============================================================================

int task_wolfpack_set_en_speed_loop(int v)   { en_speed_loop   = v; return SUCCESS; }
int task_wolfpack_set_en_mtpa(int v)         { en_mtpa         = v; return SUCCESS; }
int task_wolfpack_set_en_fw(int v)           { en_fw           = v; return SUCCESS; }
int task_wolfpack_set_en_state_fb(int v)     { en_state_fb     = v; return SUCCESS; }
int task_wolfpack_set_en_current_loop(int v) { en_current_loop = v; return SUCCESS; }
int task_wolfpack_set_en_vector_limit(int v) { en_vector_limit = v; return SUCCESS; }

// ============================================================================
// STATISTICS
// ============================================================================

void task_wolfpack_stats_print(void) { task_stats_print(&tcb.stats); }
void task_wolfpack_stats_reset(void) { task_stats_reset(&tcb.stats); }

#endif // APP_WOLFPACK
