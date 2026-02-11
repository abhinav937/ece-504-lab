
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

//#define USER_ENCODER_PULSES_PER_REV_BITS (14)
//#define USER_ENCODER_PULSES_PER_REV (1 << USER_ENCODER_PULSES_PER_REV_BITS)

#define ENCODER_COUNTS (20000.0)			//  Number of Encoder Counts per revolution
#define ENCODER_COUNTS_INV 1.0*(1/ENCODER_COUNTS) // Inverse of encoder counts per mechanical revolution
#define THETA_M_OFFSET 1.0*(0)			// Offset angle added to raw encoder angle [rad]
#define ENCODER_OFFSET (PIOVER2)		// Offset of AMDS encoder1 [rad/count]

#define HV_SENSE_GAIN (0.0163)			// Gain of AMDS HV Sense, ch_1 [V/count]
#define HV_SENSE_OFFSET_COUNTS (32768)	// HV Sense nominal offset [counts]
#define CURRENT_SENSE_GAIN (0.002028) 	// Current sensor gain [A/count]
#define CURRENT_SENSE_OFFSET_COUNTS (36572)//(28917)	// Current sensor nominal offset in counts

#define CALIB_SAMPLES (10000)			// Number of control periods to sample analog sensors and perform filtered offset calc.

#define EXP_W_F_TS 1.0*(0.9937)	 		// Digital low pass filter coefficient = exp(-2*3.14159*(fc)*Ts);  0.9937 for fc = 10 Hz;
#define ONE_OVER_PI2 (0.15915494309)	// 1/(2*pi)

#define POLE_PAIRS (4.0)					// Number of pole pairs
#define POLE_PAIRS_INV (1/POLE_PAIRS)	// Inverse of number of pole pairs

#define PM_FLUX_V_SEC_PER_RAD (0.05)	// Flux constant of PM in Volts per ELECTRICAL rad/s
#define VOLT_PER_HZ_V_INITAL (2)		// Volts/Hz command at zero frequency [V] - optional

#define WOLFPACK_VOLTAGE_MAX (40)		// Over-voltage protection level for Wolfpack Inverter
#define WOLFPACK_CURRENT_MAX (10)		// Over-current protection level for Wolfpack Inverter

#define VOLTSPERHZ_RPM_PER_SEC (100)	// Volts/Hz command rate limit [RPM/sec]
#define VOLTSPERHZ_RPM_LIMIT (1000)		// Volts/Hz absolute limit [RPM]
#define ENABLE_VOLTSPERHZ (1)			// Set to 1 to enable Volts/Hz output [Boolean]

const uint8_t amds_port = 1;
int LOG_amds_valid = 0;					// Status of AMDS data

double Ts = 1.0 / (double) TASK_WOLFPACK_UPDATES_PER_SEC;

// ****** State machine and related variables

int LOG_wolf_state = 1;					// Wolfpack code state machine present value.  0 = CALIBRATE, 1 = IDLE, 2 = RUNNING, 3 = TRIPPED
int wolf_state_prev = 1;				// Wolfpack code state machine previous value.  0 = CALIBRATE, 1 = IDLE, 2 = RUNNING, 3 = TRIPPED
int sm_request_idle = 0;				// Wolfpack code state machine IDLE state request. 0 = IDLE state NOT requested, 1 = IDLE state requested
int sm_request_run = 0;					// Wolfpack code state machine RUN state request. 0 = RUN state NOT requested, 1 = RUN state requested
int sm_request_calibrate = 0;			// Wolfpack code state machine CALIBRATE state request. 0 = CALIBRATE state NOT requested, 1 = CALIBRATE state requested
int sm_request_trip_clear = 0;			// Wolfpack code state machine TRIP state request. 0 = TRIP state NOT requested, 1 = TRIP state requested

int calibrate_status = 0;				// Calibration status (1 = in the process of calibrating sensors, 0 otherwise)
int calibrate_count = 0;				// Current calibration sample count

int LOG_pwm_state = 0;					// PWM modulator block status. 0 for disabled, 1 for enabled.

// *********  Protection variables ***************************
int LOG_OV_status_dc = 0;				// Status of dc voltage relative to over-voltage trip
int LOG_OC_status_a = 0;				// Status of Phase A current relative to over-current trip level
int LOG_OC_status_b = 0;				// Status of Phase B current relative to over-current trip level
int LOG_OC_status_c = 0;				// Status of Phase C current relative to over-current trip level
int LOG_protection_status = 0;			// Protection logic used to stop state machine if signals are out of expected bounds

// ************ COMMANDs for rotor speed in Volts/Hz mode

double LOG_w_m_RPM_ref_delta_per_sec = 0;
double LOG_w_m_RPM_ref_max = 0;
double LOG_w_m_RPM_ref_limited = 0;
double LOG_w_m_RPM_ref = 0;

double V_mag_V_per_Hz = 0;				// Volts/Hz voltage magnitude command [V]
double w_e_V_per_Hz = 0;				// Volts/Hz voltage frequency command [rad/s]

// ************ COMMANDS for manual voltage control mode

double V_mag_manual = 1;				// Manual voltage magnitude command [V]
double w_e_manual = PI2*2;				// Manual voltage frequency command [rad/s]
double w_e_manual_limited = 0;			// Manual frequency command after rate limiting [rad/s]

double LOG_V_mag_cmd = 0;				// Manual or Volts/Hz - this signal used for open loop voltage command
double LOG_w_e_cmd = 0;					// Manual or Volts/Hz - this signal used for open loop frequency command
double LOG_theta_e_cmd = 0;				// Manual or Volts/Hz - this signal used for open loop angle, derived from frequency command
double theta_e_cmd_prev = 0;			// Manual or Volts/Hz - this signal is angle from previous ISR period

// ************* AC voltage inverter output variables and related modulation, duty commands

double LOG_v_cmd_a = 0;					// abc voltage commands to inverter
double LOG_v_cmd_b = 0;
double LOG_v_cmd_c = 0;
double LOG_v_cmd_ab = 0;				// a-b L-L voltage command
double LOG_m_cmd_a = 0;					// abc modulation commands to inverter (after scaling voltage commands by Vdc)
double LOG_m_cmd_b = 0;
double LOG_m_cmd_c = 0;
double LOG_m_cmd_ab = 0;				// a-b L-L modulation command
double LOG_m_0 = 0;						// Common mode modulation signal to be added to each abc phase modulation command in SVPWM or DPWM.

double LOG_duty_a = 0.5;				// abc duty ratio commands
double LOG_duty_b = 0.5;
double LOG_duty_c = 0.5;

// *************  Rotor angle, speed, and scaling, etc. variables *************
double LOG_theta_m = 0;					// Rotor angle, mech [rad]
double LOG_theta_e = 0;					// Rotor angle, elec [rad]

double theta_m_prev = 0;   				// Rotor angle, mech, from last ISR [rad]
double LOG_delta_theta_m = 0;			// Incremental rotor angle, mech [rad];

double LOG_w_m = 0;						// Rotor speed, mech [rad/s]
double LOG_w_m_RPM = 0;					// Rotor speed, mech [RPM]
double LOG_w_m_RPM_filtered = 0;		// Rotor speed, mech, after digital filter [RPM]
double LOG_w_m_filtered = 0;			// Rotor speed, mech, after digital filter [rad/s]
double LOG_w_e_filtered = 0;			// Rotor speed, elec, after digital filter [rad/s]

// Encoder and Analog Inputs AMDS raw counts from each channel (Card)
uint32_t LOG_enc_pos_data = 0;			// Encoder counts
uint32_t LOG_amds_ch_1_data = 0;		// DC Bus voltage, counts
uint32_t LOG_amds_ch_2_data = 0;		// Current, Phase A, counts
uint32_t LOG_amds_ch_4_data = 0;		// Current, Phase B, counts

// ********* Analog Sense Variables and Offsets
double LOG_v_dc = 0;					// HV Voltage Sense [V]
double LOG_v_dc_offset = 0;				// Offset of AMDS HV Sense, ch_1 [V]

double LOG_i_a = 0;						// AMDS Phase A current [A]
double LOG_i_b = 0;						// AMDS Phase B current [A]
double LOG_i_c = 0;						// AMDS Phase C current [A]
double LOG_i_a_offset = 0;				// Offset of AMDS Current A [A]
double LOG_i_b_offset = 0;				// Offset of AMDS Current B [A]

// Metrics for tracking control loop execution, ADC sampling etc.
double LOG_control_looptime = 0;
double LOG_control_runtime = 0;
static uint32_t last_now_start = 0;
static task_control_block_t tcb;  			// Scheduler TCB which holds task "context"

int task_wolfpack_init(void)
{
    if (scheduler_tcb_is_registered(&tcb)) {
        return FAILURE;
    }
    // Fill TCB with parameters
    scheduler_tcb_init(&tcb, task_wolfpack_callback, NULL, "wolfpack", TASK_WOLFPACK_INTERVAL_USEC);

    // Enable statistics for this task
    //task_stats_enable(&tcb.stats);

    // Register task with scheduler
    return scheduler_tcb_register(&tcb);
    // return scheduler_tcb_register_high_priority(&tcb);
}

int task_wolfpack_deinit(void)
{
	pwm_disable();
    return scheduler_tcb_unregister(&tcb);
}

// Rate limiting helper function
// Applies rate limiting to a signal with specified max rate of change
static inline double rate_limit(double target, double current, double max_delta)
{
	double delta = target - current;

	if (delta >= max_delta) {
		return current + max_delta;
	} else if (delta <= -max_delta) {
		return current - max_delta;
	} else {
		return target;
	}
}

void task_wolfpack_callback(void *arg)
{
    // Compute and log the loop time for this task
    uint32_t now_start = cpu_timer_now();
    uint32_t looptime = now_start - last_now_start;
    last_now_start = now_start;
    LOG_control_looptime = cpu_timer_ticks_to_usec(looptime);

    // Sample encoder (from AMDC directly) and analog inputs (from AMDS sensor cards)
	encoder_get_position(&LOG_enc_pos_data);			// Encoder position from FPGA on AMDC
	// Sample AMDS data
    amds_get_data(1, AMDS_CH_1, &LOG_amds_ch_1_data);	// HV Voltage Sensor
    amds_get_data(1, AMDS_CH_2, &LOG_amds_ch_2_data);	// Current Sensor, Phase A
    amds_get_data(1, AMDS_CH_4, &LOG_amds_ch_4_data);	// Current Sensor, Phase B
    LOG_amds_valid = amds_check_data_validity(1);

	// Convert encoder angle counts radians
	LOG_theta_m = (double) PI2 * (LOG_enc_pos_data) * (double)ENCODER_COUNTS_INV + (double)THETA_M_OFFSET;

	// ********** Logic to ensure that theta_m is bound between 0 and 2*PI. **************
	if(LOG_theta_m < 0)
	{				// If theta_m is a negative number, add integer number (floor) 2*PI to yield positive theta_m
		LOG_theta_m += PI2*fabs(floor(LOG_theta_m * ONE_OVER_PI2 ));
	}
	LOG_theta_m = fmod(LOG_theta_m, PI2); 							// Wrap theta_m to 0 to 2*PI;
	LOG_theta_e = LOG_theta_m * POLE_PAIRS;							// Convert from mechanical to electrical angle
	LOG_theta_e = fmod(LOG_theta_e + PI2, PI2);  					// Wrap theta_e to 0 to 2*PI;

	// ********** Logic to ensure change in theta_m (delta_theta_m) is bound between + and - PI when rolling over. **************
	LOG_delta_theta_m = LOG_theta_m - theta_m_prev;					// Logic to properly deal with roll-over of increment in angle
	if (LOG_delta_theta_m < -PI){
		LOG_delta_theta_m = LOG_theta_m - theta_m_prev + PI2;		// If delta_theta_m is <-PI, add 2*PI to it.
	}
	else if (LOG_delta_theta_m > PI){
		LOG_delta_theta_m = LOG_theta_m - theta_m_prev - PI2;		// If delta_theta_m is >PI, subtract 2*PI from it.
	}
	else { } 														// No need to change delta_theta_m if it's already within -PI to +PI;

	// ****** Math to calculate speed from difference in angle b/w control periods, filter the speed due to angle chatter, convert to rad/s, rad/s electrical, etc.
	LOG_w_m = LOG_delta_theta_m * TASK_WOLFPACK_UPDATES_PER_SEC;	// Mechanical speed, [rad/s]
	LOG_w_m_RPM = RAD_PER_SEC_TO_RPM(LOG_w_m);						// Convert speed [rad/s] to [RPM]
	LOG_w_m_RPM_filtered = (1-EXP_W_F_TS)*LOG_w_m_RPM + LOG_w_m_RPM_filtered*EXP_W_F_TS;  // Filtered mechanical speed [RPM]
	LOG_w_m_filtered = RPM_TO_RAD_PER_SEC(LOG_w_m_RPM_filtered);	// Filtered mechanical freq [rad/s]
	LOG_w_e_filtered = POLE_PAIRS * LOG_w_m_filtered;				// Filtered electrical freq [rad/s]

	// ******************* Read AMDC and Encoder sensors
	LOG_v_dc = HV_SENSE_GAIN*((LOG_amds_ch_1_data*1.0) - HV_SENSE_OFFSET_COUNTS);  //  Voltage sense data
    LOG_i_a = CURRENT_SENSE_GAIN*((LOG_amds_ch_2_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS);   //  Current sense data, Phase A
    LOG_i_b = CURRENT_SENSE_GAIN*((LOG_amds_ch_4_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS);   //  Current sense data, Phase B
    LOG_i_c = -(LOG_i_a + LOG_i_b);

	// ******************* Compare voltage and current measurements with protection limits, set protection status if necessary (cause State Machine to TRIP)
	LOG_OV_status_dc = (fabs(LOG_v_dc) > WOLFPACK_VOLTAGE_MAX);
    LOG_OC_status_a = (fabs(LOG_i_a) > WOLFPACK_CURRENT_MAX);
	LOG_OC_status_b = (fabs(LOG_i_b) > WOLFPACK_CURRENT_MAX);
	LOG_OC_status_c = (fabs(LOG_i_c) > WOLFPACK_CURRENT_MAX);
	LOG_protection_status = (LOG_OC_status_a == 1) || (LOG_OC_status_b == 1) || (LOG_OC_status_c == 1) || (LOG_OV_status_dc == 1);

	// *******************  STATE MACHINE ******************************

	switch (LOG_wolf_state)
	{
	case 0: // ************* CALIBRATE ******************
		LOG_pwm_state = pwm_disable(); 	// Ensure that PWMs are disabled
		if (!calibrate_status) 			// Transition logic
		{
			calibrate_count++;  		// Increment calibration counter
		}
		// Clear all speed references and ramping variables
		LOG_w_m_RPM_ref_limited = 0;
		LOG_w_m_RPM_ref = 0;
		w_e_manual_limited = 0;
	    LOG_v_dc_offset = (1.0-EXP_W_F_TS)*LOG_v_dc + LOG_v_dc_offset*EXP_W_F_TS;  // Low pass filter Voltage Sensor Input
	    LOG_i_a_offset = (1.0-EXP_W_F_TS)*LOG_i_a + LOG_i_a_offset*EXP_W_F_TS;  // Low pass filter Current Sensor A Input
	    LOG_i_b_offset = (1.0-EXP_W_F_TS)*LOG_i_b + LOG_i_b_offset*EXP_W_F_TS;  // Low pass filter Current Sensor B Input

		if (calibrate_count >= CALIB_SAMPLES)
		{
			calibrate_status = 1;	// Record that calibration completed
			LOG_wolf_state = 1; 	// Transition to IDLE when calibration is complete
		}
	    break;

	case 1: // ************* IDLE ******************
		LOG_pwm_state = pwm_disable(); 	// Ensure that PWMs are disabled
		LOG_w_m_RPM_ref_limited = 0;	// Clear Volt/Hz limited reference
		LOG_w_m_RPM_ref = 0;			// Clear Volt/Hz speed reference
		w_e_manual_limited = 0;			// Clear manual mode limited frequency

		if (LOG_protection_status)		// Transition to TRIPPED if protections are active.
		{
			LOG_wolf_state = 3;
		}
		else if (sm_request_calibrate == 1) // Otherwise to CALIBRATE if requested
		{
			LOG_wolf_state = 0;			// Transition to Calibrate
			calibrate_status = 0;		// Zero the Calibration Status
			calibrate_count = 0;		// Zero the Calibration Counter
		}
		else if (sm_request_run == 1)	// Otherwise to RUNNING if requested
		{
			LOG_wolf_state = 2;
		}
		break;

	case 2: // ************* RUNNING ******************
		if (LOG_wolf_state != wolf_state_prev)
		{
			// Any activities to do immediately upon entry to RUNNING state
		}
		if (LOG_protection_status)		// Transition to TRIPPED if protections are active.
		{
			LOG_wolf_state = 3;
		}
		else if (sm_request_idle == 1)	// Otherwise to IDLE if requested
		{
			LOG_wolf_state = 1;
		}
		LOG_pwm_state = pwm_enable();				// Enable PWMs
		break;

	default: // ************* TRIPPED ******************
		LOG_pwm_state = pwm_disable();					// Ensure that PWMs are disabled
		if (LOG_protection_status == 0 && sm_request_trip_clear == 1) // Transition to IDLE if protections are NOT active AND clear requested
		{
			sm_request_trip_clear = 0; 				// Reset trip clear request.
			LOG_wolf_state = 1;						// Transition to Idle if protections are OK and trip_clear requested.
		}
		break;
	}

	// ****************** Cleanup Uncleared Commands
	sm_request_trip_clear = 0;		// Clear Trip Clear request
	sm_request_run = 0;				// Clear Run request
	sm_request_idle = 0;			// Clear Idle request
	sm_request_calibrate = 0;		// Clear Calibrate request

	// ******************* End of State Machine  ****************

	// ******************* Read AMDC and Encoder sensors - NOW with updated offsets from Calibration state.
	LOG_v_dc = HV_SENSE_GAIN*((LOG_amds_ch_1_data*1.0) - HV_SENSE_OFFSET_COUNTS) - LOG_v_dc_offset;   			//  Voltage sense data
	LOG_i_a = CURRENT_SENSE_GAIN*((LOG_amds_ch_2_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS) - LOG_i_a_offset;   //  Current sense data, Phase A
	LOG_i_b = CURRENT_SENSE_GAIN*((LOG_amds_ch_4_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS) - LOG_i_b_offset;   //  Current sense data, Phase B
	LOG_i_c = -(LOG_i_a + LOG_i_b);

	if (ENABLE_VOLTSPERHZ) 	// Select which function to use to determine output inverter commands
	{		// Inverter voltage commands come from Volts/Hz algorithm

	    //*************** Insert Volts/Hertz Filtering Logic here... ****************
		// Apply rate limiting
		double max_delta = VOLTSPERHZ_RPM_PER_SEC * Ts;
		LOG_w_m_RPM_ref_limited = rate_limit(LOG_w_m_RPM_ref, LOG_w_m_RPM_ref_limited, max_delta);

		// Apply magnitude limit
		if (LOG_w_m_RPM_ref_limited > VOLTSPERHZ_RPM_LIMIT) {
		    LOG_w_m_RPM_ref_limited = VOLTSPERHZ_RPM_LIMIT;
		} else if (LOG_w_m_RPM_ref_limited < -VOLTSPERHZ_RPM_LIMIT) {
		    LOG_w_m_RPM_ref_limited = -VOLTSPERHZ_RPM_LIMIT;
		}
		w_e_V_per_Hz = POLE_PAIRS * RPM_TO_RAD_PER_SEC(LOG_w_m_RPM_ref_limited);  		// Convert V/Hz RPM command to we command [rad/s]
		V_mag_V_per_Hz = w_e_V_per_Hz * PM_FLUX_V_SEC_PER_RAD + VOLT_PER_HZ_V_INITAL;	// Create V/Hz Voltage command from we command and estimate of flux [V]

		LOG_V_mag_cmd = V_mag_V_per_Hz;													// Assign V/Hz V command to inverter V command
		LOG_w_e_cmd = w_e_V_per_Hz;														// Assign V/Hz we command to inverter we command
		}
	else	// Inverter voltage commands come from "manual" variables sent from Python script
		{
		// Convert manual frequency to RPM equivalent for rate limiting
		double w_e_manual_RPM_equiv = RAD_PER_SEC_TO_RPM(w_e_manual / POLE_PAIRS);
		double w_e_manual_limited_RPM_equiv = RAD_PER_SEC_TO_RPM(w_e_manual_limited / POLE_PAIRS);

		// Apply rate limiting in RPM
		double max_delta_manual = VOLTSPERHZ_RPM_PER_SEC * Ts;
		w_e_manual_limited_RPM_equiv = rate_limit(w_e_manual_RPM_equiv, w_e_manual_limited_RPM_equiv, max_delta_manual);

		// Convert back to rad/s electrical
		w_e_manual_limited = POLE_PAIRS * RPM_TO_RAD_PER_SEC(w_e_manual_limited_RPM_equiv);

		LOG_V_mag_cmd = V_mag_manual;													// Assign Manual V command to inverter V command
		LOG_w_e_cmd = w_e_manual_limited;												// Assign Manual we command (rate limited) to inverter we command
		}

	LOG_theta_e_cmd = theta_e_cmd_prev + LOG_w_e_cmd * Ts;		// Increment the theta e command
	LOG_theta_e_cmd = fmod(LOG_theta_e_cmd + PI2, PI2);			// Modulus the theta e command by 2*PI

	LOG_v_cmd_a = LOG_V_mag_cmd*cos(LOG_theta_e_cmd);			// Phase A voltage cmd = sinusoid with Mag and Freq
	LOG_v_cmd_b = LOG_V_mag_cmd*cos(LOG_theta_e_cmd - PI2/3);	// Phase B voltage cmd = sinusoid with Mag and Freq
	LOG_v_cmd_c = LOG_V_mag_cmd*cos(LOG_theta_e_cmd + PI2/3);	// Phase C voltage cmd = sinusoid with Mag and Freq
	LOG_v_cmd_ab = LOG_v_cmd_a - LOG_v_cmd_b;					// Line-to-Line AB voltage command

	// Scale voltage command by Vdc/2 for sine PWM
    LOG_m_cmd_a = 2*LOG_v_cmd_a/LOG_v_dc;						// Phase A modulation cmd from scaled Phase A voltage cmd
    LOG_m_cmd_b = 2*LOG_v_cmd_b/LOG_v_dc;						// Phase B modulation cmd from scaled Phase B voltage cmd
    LOG_m_cmd_c = 2*LOG_v_cmd_c/LOG_v_dc;						// Phase C modulation cmd from scaled Phase C voltage cmd

    //*************** Insert space vector modulation calculations here... ****************
    // SVPWM CODE START - ADD THIS CODE
    // Find the maximum and minimum of the three phase modulation commands
    double m_max = LOG_m_cmd_a;
    double m_min = LOG_m_cmd_a;

    if (LOG_m_cmd_b > m_max) {
        m_max = LOG_m_cmd_b;
    }
    if (LOG_m_cmd_c > m_max) {
        m_max = LOG_m_cmd_c;
    }

    if (LOG_m_cmd_b < m_min) {
        m_min = LOG_m_cmd_b;
    }
    if (LOG_m_cmd_c < m_min) {
        m_min = LOG_m_cmd_c;
    }

    // Calculate zero-sequence modulation to center the waveform
    // This maximizes DC bus utilization and implements SVPWM
    LOG_m_0 = -0.5 * (m_max + m_min);
    // SVPWM CODE END

    LOG_m_cmd_a = LOG_m_cmd_a + LOG_m_0;						// Phase A modulation command with any common mode m_0
    LOG_m_cmd_b = LOG_m_cmd_b + LOG_m_0;						// Phase B modulation command with any common mode m_0
    LOG_m_cmd_c = LOG_m_cmd_c + LOG_m_0;						// Phase C modulation command with any common mode m_0

    LOG_m_cmd_ab = LOG_m_cmd_a - LOG_m_cmd_b;					// Line-to-Line AB modulation command

    // Level shift modulation command by 0.5 for individual phase leg duty ratio 0 <= d <= 1.0
    LOG_duty_a = 0.5*(1 + LOG_m_cmd_a);								// Phase A duty ratio calculation
    LOG_duty_b = 0.5*(1 + LOG_m_cmd_b);								// Phase B duty ratio calculation
    LOG_duty_c = 0.5*(1 + LOG_m_cmd_c);								// Phase C duty ratio calculation

    // Update PWM peripheral in FPGA
    pwm_set_duty(0, LOG_duty_a); 								// Set HB1 duty ratio (INV1, PWM1 and PWM2)
    pwm_set_duty(1, LOG_duty_b); 								// Set HB2 duty ratio (INV1, PWM3 and PWM4)
    pwm_set_duty(2, LOG_duty_c); 								// Set HB3 duty ratio (INV1, PWM5 and PWM6)

    // ***********  Prepare for next ISR ****************************
    theta_m_prev = LOG_theta_m;									// Assign present theta_m to theta_m_prev for use in the next ISR
    theta_e_cmd_prev = LOG_theta_e_cmd;							// Assign present theta_e_cmd to theta_e_prev for use in the next ISR
    wolf_state_prev = LOG_wolf_state;							// Assign present state to previous

    // Compute and log the run time for this task
    uint32_t now_end = cpu_timer_now();
    uint32_t runtime = now_end - now_start;
    LOG_control_runtime = cpu_timer_ticks_to_usec(runtime);
    /* End of ISR */
}



void task_wolfpack_sm_run(void)
{
	sm_request_run = 1;
}

void task_wolfpack_sm_idle(void)
{
	sm_request_idle = 1;
}

void task_wolfpack_sm_calibrate(void)
{
	sm_request_calibrate = 1;
	calibrate_status = 0;
}

void task_wolfpack_sm_trip_clear(void)
{
	sm_request_trip_clear = 1;
}

int task_wolfpack_sm_get_state(void)
{
	return LOG_wolf_state;
}

int task_wolfpack_set_manual_w_e(double w_e)
{
	w_e_manual = w_e;
    return SUCCESS;
}

int task_wolfpack_set_manual_V_mag(double Vmag)
{
    V_mag_manual = Vmag;
    return SUCCESS;
}

void task_wolfpack_set_Volt_per_Hz_speed(double speed_ref_RPM)
{
	LOG_w_m_RPM_ref = speed_ref_RPM;
}

void task_wolfpack_stats_print(void)
{
    task_stats_print(&tcb.stats);
}

void task_wolfpack_stats_reset(void)
{
    task_stats_reset(&tcb.stats);
}

#endif // APP_WOLFPACK
