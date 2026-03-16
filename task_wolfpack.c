
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

#define PM_FLUX_V_SEC_PER_RAD (0.0383)		// Flux constant of PM in Volts per ELECTRICAL rad/s

#define WOLFPACK_VOLTAGE_MAX (70)		// Over-voltage protection level for Wolfpack Inverter
#define WOLFPACK_CURRENT_MAX (12)		// Over-current protection level for Wolfpack Inverter
#define	VECTOR_CURRENT_LIMIT (2)		// Current Vector Command Limit (Set to 8 for normal operation)

#define L_DS_ESTIMATE (0.001)			// Stator d axis inductance estimate [H]
#define L_QS_ESTIMATE (0.0016)			// Stator q axis inductance estimate [H]
#define R_S_ESTIMATE (0.55)				// Stator resistance estimate [Ohms]

#define IREG_W_GCF (1885)				//
#define IREG_W_PI_CROSS_OVER (500)		//
#define IREG_KPD (0.001 * IREG_W_GCF)
#define IREG_KPQ (0.0016 * IREG_W_GCF)
#define IREG_KID (IREG_W_PI_CROSS_OVER * IREG_KPD)
#define IREG_KIQ (IREG_W_PI_CROSS_OVER * IREG_KPQ)

// Prelab 5: Current Regulator Selection
// Comment out to use Classical PI (no decoupling) - Test 2.0
// Uncomment to use State Feedback Decoupling - Tests 2.1.1, 2.1.2
//#define STATE_FEEDBACK_DECOUPLING

// Lab 4-1: Speed Regulator Parameters
// Update J_ESTIMATE and B_ESTIMATE after measuring them in test 1.1.
// Adjust SPEED_REG_W_GCF for each bandwidth trial (2*pi*1, 2*pi*pi, 2*pi*10).
#define J_ESTIMATE        (0.0042668)                  // Rotational inertia estimate [kg*m^2]
#define B_ESTIMATE        (0.0020483)                  // Rotational damping estimate [N*m*s/rad]
#define SPEED_REG_W_GCF   (6.28318530718)        // Speed regulator gain crossover frequency [rad/s] (default: 2*pi*1 Hz)
#define Kp_w_m            (J_ESTIMATE * SPEED_REG_W_GCF)   // Speed PI proportional gain [N*m*s/rad]
#define Ki_w_m            (B_ESTIMATE * SPEED_REG_W_GCF)   // Speed PI integral gain [N*m/rad]

// Lab 4-2: Field Weakening Regulator Parameters
#define W_E_NOM_FW       (500.0)                                        // Nominal elec. speed for FW KI tuning [rad/s elec]
#define FW_REG_KI        (SPEED_REG_W_GCF / (W_E_NOM_FW * L_DS_ESTIMATE)) // FW integrator gain [1/H]
#define FW_DC_TO_AC_GAIN (0.95 / 1.73205080757)                         // k/sqrt(3): DC-to-AC voltage reference gain


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


// ************* AC voltage inverter output variables and related modulation, duty commands

double LOG_v_cmd_a = 0;					// abc voltage commands to inverter
double LOG_v_cmd_b = 0;
double LOG_v_cmd_c = 0;
double LOG_v_cmd_ab = 0;				// a-b L-L voltage command
double LOG_m_cmd_a = 0;					// abc modulation commands to inverter (after scaling voltage commands by Vdc)
double LOG_m_cmd_b = 0;
double LOG_m_cmd_c = 0;
double LOG_m_cmd_ab = 0;				// a-b L-L modulation command
double LOG_m_cmd_0 = 0;						// Common mode modulation signal to be added to each abc phase modulation command in SVPWM or DPWM.

double LOG_duty_a = 0.5;				// abc duty ratio commands
double LOG_duty_b = 0.5;
double LOG_duty_c = 0.5;

// *************  Rotor angle, speed, and scaling, etc. variables *************
double LOG_theta_m = 0;					// Rotor angle, mech [rad]
double LOG_theta_e = 0;					// Rotor angle, elec [rad]
double theta_e_offset = -1.047197;		// Offset angle added to theta_e for reference frame [rad]

double theta_m_prev = 0;   				// Rotor angle, mech, from last ISR [rad]
double LOG_delta_theta_m = 0;			// Incremental rotor angle, mech [rad];

double LOG_w_m = 0;						// Rotor speed, mech [rad/s]
double LOG_w_m_RPM = 0;					// Rotor speed, mech [RPM]
double LOG_w_m_RPM_filtered = 0;		// Rotor speed, mech, after digital filter [RPM]
double LOG_w_m_filtered = 0;			// Rotor speed, mech, after digital filter [rad/s]
double LOG_w_e_filtered = 0;			// Rotor speed, elec, after digital filter [rad/s]

double LOG_theta_e_ref_frame = 0;		// Angle to be used in Park Transformations

// Encoder and Analog Inputs AMDS raw counts from each channel (Card)
uint32_t LOG_enc_pos_data = 0;			// Encoder counts
uint32_t LOG_amds_ch_1_data = 0;		// DC Bus voltage, counts
uint32_t LOG_amds_ch_2_data = 0;		// Current, Phase A, counts
uint32_t LOG_amds_ch_4_data = 0;		// Current, Phase B, counts
uint32_t LOG_amds_ch_6_data = 0;		// DC Bus current sensor, counts

// ********* Analog Sense Variables and Offsets
double LOG_v_dc = 0;					// HV Voltage Sense [V]
double LOG_v_dc_offset = 0;				// Offset of AMDS HV Sense, ch_1 [V]

double LOG_i_a = 0;						// AMDS Phase A current [A]
double LOG_i_b = 0;						// AMDS Phase B current [A]
double LOG_i_c = 0;						// AMDS Phase C current [A]
double LOG_i_a_offset = 0;				// Offset of AMDS Current A [A]
double LOG_i_b_offset = 0;				// Offset of AMDS Current B [A]

// Lab 4-2: DC Bus Current (AMDS channel 6)
double LOG_i_dc = 0;					// DC bus current [A]
double LOG_i_dc_offset = 0;				// DC bus current sensor offset [A]

// Lab 4-2: Field Weakening Regulator variables
double LOG_v_dq_mag_cmd = 0;			// Voltage magnitude reference (FW cmd) [V]
double LOG_v_dq_mag = 0;				// Voltage magnitude from previous ISR cmd [V]
double LOG_v_dq_mag_error_inte = 0;		// FW integrator state [V*s]
double LOG_i_d_fw = 0;					// FW regulator d-axis current output [A]

// Lab 4-2: Power estimates
double LOG_p_dc = 0;					// DC input power [W]
double LOG_p_ac = 0;					// AC stator power [W]
double LOG_p_mech = 0;					// Mechanical shaft power [W]

// *********  Array of abc, dq0 signals - put into arrays to pass to transform functions *****************
double i_abc[3] = {0,0,0};				// abc current vectors (3 element long)
double i_dq0[3] = {0,0,0};				// dq0 current vectors (3 element long)
double v_cmd_abc[3] = {0,0,0};			// abc command vectors (3 element long)
double v_cmd_dq0[3] = {0,0,0};			// dq0 command vectors (3 element long)

double LOG_i_q = 0;						// Resulting qd0 currents for Logging
double LOG_i_d = 0;
double LOG_i_0 = 0;

double LOG_torque_estimate = 0;			// Estimated torque

double LOG_i_s_ref = 0;					// MTPA current magnitude reference (computed from speed PI torque command)

// Lab 4-1: Speed PI regulator variables
double LOG_w_m_ref = 0;				// Rotor speed reference [rad/s]
double LOG_T_e_cmd_prop = 0;			// Speed PI proportional torque command [N*m]
double LOG_T_e_cmd_inte = 0;			// Speed PI integral torque command [N*m]
double LOG_T_e_cmd = 0;				// Total speed PI torque command [N*m]
double w_m_error = 0;					// Speed error (ref - actual) [rad/s]

double LOG_i_q_ref_manual = 0;			// Manual input qd current references
double LOG_i_d_ref_manual = 0;			// These are both reset in the IDLE state

double LOG_i_q_mtpa = 0;				// dq current references after vector limiting block
double LOG_i_d_mtpa = 0;				//

double LOG_i_q_ref = 0;					// dq total current references
double LOG_i_d_ref = 0;					//

double LOG_i_q_ref_limited = 0;			// dq current references after vector limiting block
double LOG_i_d_ref_limited = 0;			//

double LOG_i_q_Error = 0;				// Difference between respective current references and feedback currents
double LOG_i_d_Error = 0;

double LOG_i_q_Error_Integral = 0;  	// Integral error of q and d axis current references.
double LOG_i_d_Error_Integral = 0;		// These are both reset in the IDLE state

double LOG_v_cmd_q_BEMF = 0;			// Back EMF voltage in q-axis
double LOG_v_cmd_q_Prop = 0;		// Kp * Error term in q-axis
double LOG_v_cmd_q_Inte = 0;	// Ki * Integral of Error in q-axis
double LOG_v_cmd_q = 0;					// Total voltage command in q-axis

double LOG_v_cmd_d_BEMF = 0;			// Back EMF voltage in d-axis
double LOG_v_cmd_d_Prop = 0;		// Kp * Error term in d-axis
double LOG_v_cmd_d_Inte = 0;	// Ki * Integral of Error in d-axis
double LOG_v_cmd_d = 0;					// Total voltage command in d-axis

double LOG_v_cmd_0 = 0;					// Zero sequence voltage command

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
    amds_get_data(1, AMDS_CH_6, &LOG_amds_ch_6_data);	// Current Sensor, DC Bus
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

	// ******************* Read AMDC and Encoder sensors - NOW with updated offsets from Calibration state.
	if(LOG_wolf_state != 0){   // If not in CALIBRATE state use the offset
		LOG_v_dc = HV_SENSE_GAIN*((LOG_amds_ch_1_data*1.0) - HV_SENSE_OFFSET_COUNTS) - LOG_v_dc_offset;   			//  Voltage sense data
		LOG_i_a = CURRENT_SENSE_GAIN*((LOG_amds_ch_2_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS) - LOG_i_a_offset;   //  Current sense data, Phase A
		LOG_i_b = CURRENT_SENSE_GAIN*((LOG_amds_ch_4_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS) - LOG_i_b_offset;   //  Current sense data, Phase B
		LOG_i_c = -(LOG_i_a + LOG_i_b);
		LOG_i_dc = CURRENT_SENSE_GAIN*((LOG_amds_ch_6_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS) - LOG_i_dc_offset; //  DC bus current sense data
	}
	else{						// If in CALIBRATE state DO NOT use the offset (or else it goofs the offset calc itself).
		// ******************* Read AMDC and Encoder sensors
		LOG_v_dc = HV_SENSE_GAIN*((LOG_amds_ch_1_data*1.0) - HV_SENSE_OFFSET_COUNTS);  //  Voltage sense data
		LOG_i_a = CURRENT_SENSE_GAIN*((LOG_amds_ch_2_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS);   //  Current sense data, Phase A
		LOG_i_b = CURRENT_SENSE_GAIN*((LOG_amds_ch_4_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS);   //  Current sense data, Phase B
		LOG_i_c = -(LOG_i_a + LOG_i_b);
		LOG_i_dc = CURRENT_SENSE_GAIN*((LOG_amds_ch_6_data*1.0) - CURRENT_SENSE_OFFSET_COUNTS);  //  DC bus current sense data (no offset)
	}

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
	    LOG_v_dc_offset = (1.0-EXP_W_F_TS)*LOG_v_dc + LOG_v_dc_offset*EXP_W_F_TS;  // Low pass filter Voltage Sensor Input
	    LOG_i_a_offset = (1.0-EXP_W_F_TS)*LOG_i_a + LOG_i_a_offset*EXP_W_F_TS;  // Low pass filter Current Sensor A Input
	    LOG_i_b_offset = (1.0-EXP_W_F_TS)*LOG_i_b + LOG_i_b_offset*EXP_W_F_TS;  // Low pass filter Current Sensor B Input
	    LOG_i_dc_offset = (1.0-EXP_W_F_TS)*LOG_i_dc + LOG_i_dc_offset*EXP_W_F_TS;  // Low pass filter DC Bus Current Sensor

		if (calibrate_count >= CALIB_SAMPLES)
		{
			calibrate_status = 1;	// Record that calibration completed
			LOG_wolf_state = 1; 	// Transition to IDLE when calibration is complete
		}
	    break;

	case 1: // ************* IDLE ******************
		LOG_pwm_state = pwm_disable(); 	// Ensure that PWMs are disabled

		LOG_i_q_Error_Integral = 0;		// Clear d and q current integrators
		LOG_i_d_Error_Integral = 0;
		LOG_i_d_ref_manual = 0;			// Clear manual d and q current commands
		LOG_i_q_ref_manual = 0;
		LOG_w_m_ref = 0;				// Clear speed reference and speed PI states
		LOG_T_e_cmd_prop = 0;
		LOG_T_e_cmd_inte = 0;
		LOG_T_e_cmd = 0;
		LOG_v_dq_mag_error_inte = 0;	// Clear FW integrator state
		LOG_i_d_fw = 0;					// Clear FW d-axis current output

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

	// ****************** Cleanup Uncleared Commands ****************
	sm_request_trip_clear = 0;		// Clear Trip Clear request
	sm_request_run = 0;				// Clear Run request
	sm_request_idle = 0;			// Clear Idle request
	sm_request_calibrate = 0;		// Clear Calibrate request

	// ******************* End of State Machine  ****************

	// ******************* Get ready to Clark + Park transform phase currents
	i_abc[0] = LOG_i_a;											// Assign abc currents to 3 element array
	i_abc[1] = LOG_i_b;
	i_abc[2] = LOG_i_c;

	LOG_theta_e_ref_frame = LOG_theta_e + theta_e_offset;		// Use the encoder electrical angle as the Park transform angle
	LOG_theta_e = fmod(LOG_theta_e_ref_frame + PI2, PI2);  		// Wrap theta_ref_frame to 0 to 2*PI;

	transform_dqz(0,LOG_theta_e_ref_frame,i_abc,i_dq0);			// Stationary to dq0 synchronous frame transformation (Clarke + Park)

	// i_dq0 now contains i_abc currents transformed into Park Frame.

	LOG_i_d = i_dq0[0];											// Assign results of 3 element array individual current variables
	LOG_i_q = i_dq0[1];
	LOG_i_0 = i_dq0[2];

	// Prelab 4: Torque Estimate
	// T_e = (3/2) * (poles/2) * [lambda_pm * i_q + i_q * i_d * (Ld - Lq)]
	LOG_torque_estimate = 1.5 * POLE_PAIRS
	        * (PM_FLUX_V_SEC_PER_RAD * LOG_i_q
	           + LOG_i_q * LOG_i_d * (L_DS_ESTIMATE - L_QS_ESTIMATE));

	// Lab 4-1 Prelab 4: Speed PI Regulator
	// Computes torque command from speed error using proportional and integral terms.
	// Kp_w_m = J_ESTIMATE * SPEED_REG_W_GCF, Ki_w_m = B_ESTIMATE * SPEED_REG_W_GCF.
	// Set Kp_w_m = 0 and Ki_w_m = 0 (via J_ESTIMATE=B_ESTIMATE=0) for open-loop test 1.1.
	w_m_error = LOG_w_m_ref - LOG_w_m_filtered;
	LOG_T_e_cmd_prop = Kp_w_m * w_m_error;
	LOG_T_e_cmd_inte += Ki_w_m * w_m_error * Ts;
	LOG_T_e_cmd = LOG_T_e_cmd_prop + LOG_T_e_cmd_inte;

	// Lab 4-1 Prelab 5: Compute i_s_ref from torque command for MTPA
	// T_e = (3/2) * P * lambda_pm * i_s  =>  i_s_ref = T_e_cmd / (1.5 * P * lambda_pm)
	LOG_i_s_ref = LOG_T_e_cmd / (1.5 * POLE_PAIRS * PM_FLUX_V_SEC_PER_RAD);

	// Prelab 6: Maximum Torque Per Ampere (MTPA)
	// i_d_mtpa = (lambda_pm - sqrt(lambda_pm^2 + 8*(Lq-Ld)*is_ref^2)) / (4*(Lq-Ld))
	// i_q_mtpa = sign(is_ref) * sqrt(is_ref^2 - i_d_mtpa^2)
	double mtpa_radicand = PM_FLUX_V_SEC_PER_RAD * PM_FLUX_V_SEC_PER_RAD
	        + 8.0 * (L_QS_ESTIMATE - L_DS_ESTIMATE) * LOG_i_s_ref * LOG_i_s_ref;
	LOG_i_d_mtpa = (PM_FLUX_V_SEC_PER_RAD - sqrt(mtpa_radicand))
	        / (4.0 * (L_QS_ESTIMATE - L_DS_ESTIMATE));
	double iq_mtpa_radicand = LOG_i_s_ref * LOG_i_s_ref - LOG_i_d_mtpa * LOG_i_d_mtpa;
	LOG_i_q_mtpa = (LOG_i_s_ref >= 0.0 ? 1.0 : -1.0) * sqrt(fmax(0.0, iq_mtpa_radicand));

	// Lab 4-2 Prelab 3: Field Weakening Regulator
	// Uses v_cmd_d, v_cmd_q from previous ISR (one-sample delay).
	// Integrates the voltage magnitude error; clamps integrator state <= 0
	// so i_d_fw is always <= 0 (negative d-axis current for flux weakening).
	LOG_v_dq_mag     = sqrt(LOG_v_cmd_d * LOG_v_cmd_d + LOG_v_cmd_q * LOG_v_cmd_q);
	LOG_v_dq_mag_cmd = FW_DC_TO_AC_GAIN * LOG_v_dc;
	double fw_error  = LOG_v_dq_mag_cmd - LOG_v_dq_mag;   // x[k], allowed + or -
	LOG_v_dq_mag_error_inte += Ts * fw_error;              // y[k] = y[k-1] + Ts*x[k]
	if (LOG_v_dq_mag_error_inte > 0.0) {                   // Keep integrator state <= 0
	    LOG_v_dq_mag_error_inte = 0.0;
	}
	LOG_i_d_fw = FW_REG_KI * LOG_v_dq_mag_error_inte;     // i_d_fw <= 0

	// Lab 4-2 Prelab 3: Total d and q references
	// i_d_ref = min(i_d_mtpa, i_d_fw) so FW pulls id more negative when active
	LOG_i_d_ref = fmin(LOG_i_d_mtpa, LOG_i_d_fw) + LOG_i_d_ref_manual;
	LOG_i_q_ref = LOG_i_q_mtpa + LOG_i_q_ref_manual;

	// Lab 4-2 Prelab 4 (Optional): Vector Limiter with d-axis Priority
	// d-axis (i_d_ref) is passed through first up to +/- VECTOR_CURRENT_LIMIT.
	// Remaining current budget is allocated to i_q_ref.
	{
	    double id_clamped = fmax(-VECTOR_CURRENT_LIMIT, fmin(VECTOR_CURRENT_LIMIT, LOG_i_d_ref));
	    double iq_budget  = sqrt(fmax(0.0, VECTOR_CURRENT_LIMIT * VECTOR_CURRENT_LIMIT
	                                       - id_clamped * id_clamped));
	    LOG_i_d_ref_limited = id_clamped;
	    LOG_i_q_ref_limited = (LOG_i_q_ref >= 0.0 ? 1.0 : -1.0)
	                          * fmin(fabs(LOG_i_q_ref), iq_budget);
	}

	LOG_i_d_Error = LOG_i_d_ref_limited - LOG_i_d;				// d and q-axis current error, difference of reference and feedback.
	LOG_i_q_Error = LOG_i_q_ref_limited - LOG_i_q;

	LOG_i_d_Error_Integral += LOG_i_d_Error * Ts;				// d and q-axis running integral of the current error.
	LOG_i_q_Error_Integral += LOG_i_q_Error * Ts;

#ifndef STATE_FEEDBACK_DECOUPLING
	// Prelab 5: Classical PI regulator (Test 2.0) - no cross-coupling cancellation
	LOG_v_cmd_d_BEMF = 0;
	LOG_v_cmd_q_BEMF = PM_FLUX_V_SEC_PER_RAD * LOG_w_e_filtered;
#else
	// Prelab 5: State Feedback Decoupling (Tests 2.1.1, 2.1.2)
	// Adds cross-coupling cancellation terms:
	//   v_d += -we * Lq_est * iq   (cancel d-axis cross coupling)
	//   v_q += +we * Ld_est * id   (cancel q-axis cross coupling, in addition to BEMF)
	LOG_v_cmd_d_BEMF = +LOG_w_e_filtered * L_QS_ESTIMATE * LOG_i_q;
	LOG_v_cmd_q_BEMF = -PM_FLUX_V_SEC_PER_RAD * LOG_w_e_filtered
	                   + LOG_w_e_filtered * L_DS_ESTIMATE * LOG_i_d;
#endif
	LOG_v_cmd_d_Prop = IREG_KPD * LOG_i_d_Error;
	LOG_v_cmd_d_Inte = IREG_KID * LOG_i_d_Error_Integral;
	LOG_v_cmd_d = LOG_v_cmd_d_BEMF + LOG_v_cmd_d_Prop + LOG_v_cmd_d_Inte;

	LOG_v_cmd_q_Prop = IREG_KPQ * LOG_i_q_Error;
	LOG_v_cmd_q_Inte = IREG_KIQ * LOG_i_q_Error_Integral;
	LOG_v_cmd_q = LOG_v_cmd_q_BEMF + LOG_v_cmd_q_Prop + LOG_v_cmd_q_Inte;

	// Lab 4-2 Prelab 6: Power Estimates
	LOG_p_dc   = LOG_v_dc * LOG_i_dc;
	LOG_p_ac   = 1.5 * (LOG_v_cmd_q * LOG_i_q + LOG_v_cmd_d * LOG_i_d);
	LOG_p_mech = LOG_torque_estimate * LOG_w_m_filtered;

	v_cmd_dq0[0] = LOG_v_cmd_d;									// Assign the individual d, q, 0 voltage commands to the vector elements.
	v_cmd_dq0[1] = LOG_v_cmd_q;
	v_cmd_dq0[2] = LOG_v_cmd_0;

	// ********  dq0 to abc transform
	transform_dqz_inverse(0, LOG_theta_e_ref_frame, v_cmd_abc, v_cmd_dq0);

	LOG_v_cmd_a = v_cmd_abc[0];									// Extract array components to individual logging variables
	LOG_v_cmd_b = v_cmd_abc[1];
	LOG_v_cmd_c = v_cmd_abc[2];

	LOG_v_cmd_ab = LOG_v_cmd_a - LOG_v_cmd_b;					// Line-to-Line AB voltage command

	// Scale voltage command by Vdc/2 for sine PWM
    LOG_m_cmd_a = 2*LOG_v_cmd_a/LOG_v_dc;						// Phase A modulation cmd from scaled Phase A voltage cmd
    LOG_m_cmd_b = 2*LOG_v_cmd_b/LOG_v_dc;						// Phase B modulation cmd from scaled Phase B voltage cmd
    LOG_m_cmd_c = 2*LOG_v_cmd_c/LOG_v_dc;						// Phase C modulation cmd from scaled Phase C voltage cmd

    //*************** SVPWM ****************
    LOG_m_cmd_0 = -0.5*(fmax(LOG_m_cmd_a,fmax(LOG_m_cmd_b,LOG_m_cmd_c)) + fmin(LOG_m_cmd_a,fmin(LOG_m_cmd_b,LOG_m_cmd_c)));

    LOG_m_cmd_a = LOG_m_cmd_a + LOG_m_cmd_0;						// Phase A modulation command with any common mode m_0
    LOG_m_cmd_b = LOG_m_cmd_b + LOG_m_cmd_0;						// Phase B modulation command with any common mode m_0
    LOG_m_cmd_c = LOG_m_cmd_c + LOG_m_cmd_0;						// Phase C modulation command with any common mode m_0

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


int task_wolfpack_set_i_q_ref_manual(double i)
{
	LOG_i_q_ref_manual = i;
    return SUCCESS;
}

int task_wolfpack_set_i_d_ref_manual(double i)
{
	LOG_i_d_ref_manual = i;
    return SUCCESS;
}

int task_wolfpack_set_w_m_ref(double w)
{
	LOG_w_m_ref = w;
    return SUCCESS;
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
