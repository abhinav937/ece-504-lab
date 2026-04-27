
#ifdef APP_WOLFPACK

#include "cmd_wolfpack.h"
#include "drv/pwm.h"
#include "sys/commands.h"
#include "sys/defines.h"
#include "sys/util.h"
#include "../task_wolfpack.h"
#include <stdlib.h>
#include <string.h>

// Stores command entry for command system module
static command_entry_t cmd_entry;

// Defines help content displayed for this command
// when user types "help" at command prompt

static command_help_t cmd_help[] = {
    { "init", "Start task" },
    { "deinit", "Stop task" },
	{ "sm_get_state", "Get current SM state, (0 = Idle, 1 = Calibrate, 2 = Running, 3 = Tripped" },
	{ "sm_idle", "Request SM Idle, only acknowledged from RUNNING" },
	{ "sm_calibrate", "Request SM Calibrate, only acknowledged from IDLE" },
	{ "sm_run", "Request SM Run, only acknowledged from IDLE" },
	{ "sm_trip_clear", "Request SM Trip Clear, only acknowledged from TRIPPED" },
    { "set_i_q_ref_manual", "Set q axis current manual ref [A]" },
    { "set_i_d_ref_manual", "Set d axis current manual ref [A]" },
	{ "set_w_m_ref", "Set the commanded rotor speed reference [rad/s]"},
	{ "set_theta_m_ref", "Set single-rev position reference, [0, 2pi), shortest path [rad]"},
	{ "set_theta_m_ref_abs", "Set absolute multi-turn position from encoder zero, ramped at ramp_w_max [rad]" },
	{ "set_theta_m_ref_rel", "Add delta to current position ramp target; stacks: 5pi+5pi == 10pi [rad]" },
	{ "set_pos_kp", "Set position-loop proportional gain [rad/s per rad]" },
	{ "set_pos_ki", "Set position-loop integral gain [rad/s per rad*s]" },
	{ "set_pos_w_m_ref_max", "Set position-loop speed command limit [rad/s]" },
	{ "set_mtpa_en", "Enable (1) or disable (0) MTPA d/q current decomposition" },
	{ "set_pos_vff_gain", "Set velocity feedforward gain (1.0=full, 0=off)" },
	{ "set_speed_kp", "Set speed-loop proportional gain [N*m*s/rad]" },
	{ "set_speed_ki", "Set speed-loop integral gain [N*m/rad]" },
	{ "set_ireg_kpd", "Set d-axis current-loop proportional gain [V/A]" },
	{ "set_ireg_kid", "Set d-axis current-loop integral gain [V/(A*s)]" },
	{ "set_ireg_kpq", "Set q-axis current-loop proportional gain [V/A]" },
	{ "set_ireg_kiq", "Set q-axis current-loop integral gain [V/(A*s)]" },
	{ "set_ramp_w_max", "Set position-reference ramp speed limit [rad/s] (default 50.0)" },
	{ "zero_accum", "Zero accum at current position and drive shaft to encoder zero" },
};


void cmd_wolfpack_register(void)
{
    commands_cmd_init(&cmd_entry, "wolfpack", "Some commands", cmd_help, ARRAY_SIZE(cmd_help), cmd_wolfpack);
    commands_cmd_register(&cmd_entry);
}

int cmd_wolfpack(int argc, char **argv)
{
    if (argc == 2 && STREQ("init", argv[1])) {
        if (task_wolfpack_init() != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }

    if (argc == 2 && STREQ("deinit", argv[1])) {
        if (task_wolfpack_deinit() != SUCCESS) {
            return CMD_FAILURE;
        }

        return CMD_SUCCESS;
    }

    if (argc == 2 && STREQ("sm_get_state", argv[1])) {
        int state;
      	state = task_wolfpack_sm_get_state();
      	switch (state)
      	{
      	case (0):
      		cmd_resp_printf("Current State is CALIBRATE\r\n");  //cmd_resp_printf("%i\r\n", state);
      		break;
      	case (1):
      	    cmd_resp_printf("Current State is IDLE\r\n");  //cmd_resp_printf("%i\r\n", state);
      		break;
		case (2):
	    	cmd_resp_printf("Current State is RUNNING\r\n");  //cmd_resp_printf("%i\r\n", state);
    		break;
		case (3):
	    	cmd_resp_printf("Current State is TRIPPED\r\n");  //cmd_resp_printf("%i\r\n", state);
			break;
		}
      	return CMD_SUCCESS;
    }

    if (argc == 2 && STREQ("sm_run", argv[1])) {
    	task_wolfpack_sm_run();
    	return CMD_SUCCESS;
    }

	if (argc == 2 && STREQ("sm_idle", argv[1])) {
		task_wolfpack_sm_idle();
		return CMD_SUCCESS;
	}

	if (argc == 2 && STREQ("sm_calibrate", argv[1])) {
		task_wolfpack_sm_calibrate();
		return CMD_SUCCESS;
	}

	if (argc == 2 && STREQ("sm_trip_clear", argv[1])) {
		task_wolfpack_sm_trip_clear();
		return CMD_SUCCESS;
	}

    if (argc == 3 && STREQ("set_i_q_ref_manual", argv[1])) {
        double i = strtod(argv[2], NULL);
        if (task_wolfpack_set_i_q_ref_manual(i) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_i_d_ref_manual", argv[1])) {
        double i = strtod(argv[2], NULL);
        if (task_wolfpack_set_i_d_ref_manual(i) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_w_m_ref", argv[1])) {
        double w = strtod(argv[2], NULL);
        if (task_wolfpack_set_w_m_ref(w) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_theta_m_ref", argv[1])) {
        double theta = strtod(argv[2], NULL);
        if (task_wolfpack_set_theta_m_ref(theta) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_theta_m_ref_abs", argv[1])) {
        double theta = strtod(argv[2], NULL);
        if (task_wolfpack_set_theta_m_ref_abs(theta) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_theta_m_ref_rel", argv[1])) {
        double delta = strtod(argv[2], NULL);
        if (task_wolfpack_set_theta_m_ref_rel(delta) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_pos_kp", argv[1])) {
        double kp = strtod(argv[2], NULL);
        if (task_wolfpack_set_pos_kp(kp) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_pos_ki", argv[1])) {
        double ki = strtod(argv[2], NULL);
        if (task_wolfpack_set_pos_ki(ki) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_pos_w_m_ref_max", argv[1])) {
        double w_max = strtod(argv[2], NULL);
        if (task_wolfpack_set_pos_w_m_ref_max(w_max) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_mtpa_en", argv[1])) {
        int en = (int)strtol(argv[2], NULL, 10);
        if (task_wolfpack_set_mtpa_en(en) != SUCCESS) return CMD_FAILURE;
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_pos_vff_gain", argv[1])) {
        double gain = strtod(argv[2], NULL);
        if (task_wolfpack_set_pos_vff_gain(gain) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_speed_kp", argv[1])) {
        double kp = strtod(argv[2], NULL);
        if (task_wolfpack_set_speed_kp(kp) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_speed_ki", argv[1])) {
        double ki = strtod(argv[2], NULL);
        if (task_wolfpack_set_speed_ki(ki) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_ireg_kpd", argv[1])) {
        double kp = strtod(argv[2], NULL);
        if (task_wolfpack_set_ireg_kpd(kp) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_ireg_kid", argv[1])) {
        double ki = strtod(argv[2], NULL);
        if (task_wolfpack_set_ireg_kid(ki) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_ireg_kpq", argv[1])) {
        double kp = strtod(argv[2], NULL);
        if (task_wolfpack_set_ireg_kpq(kp) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_ireg_kiq", argv[1])) {
        double ki = strtod(argv[2], NULL);
        if (task_wolfpack_set_ireg_kiq(ki) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_ramp_w_max", argv[1])) {
        double w = strtod(argv[2], NULL);
        if (task_wolfpack_set_ramp_w_max(w) != SUCCESS) return CMD_FAILURE;
        return CMD_SUCCESS;
    }

    if (argc == 2 && STREQ("zero_accum", argv[1])) {
        task_wolfpack_zero_accum();
        return CMD_SUCCESS;
    }

    return CMD_INVALID_ARGUMENTS;
}

#endif // APP_WOLFPACK
