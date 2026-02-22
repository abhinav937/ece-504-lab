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
    { "set_Ireg_w_GCF", "Set current regulator gain cross over freq (GCF) [rad/s]" },
	{ "set_Ireg_w_cross_over", "Set current regulator PI cross over freq [rad/s]" },
	{ "set_theta_e_offset", "Reference Frame offset angle [rad]" },


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

    if (argc == 3 && STREQ("set_Ireg_w_GCF", argv[1])) {
        double w = strtod(argv[2], NULL);
        if (task_wolfpack_Ireg_set_w_GCF(w) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_Ireg_w_cross_over", argv[1])) {
        double w = strtod(argv[2], NULL);
        if (task_wolfpack_Ireg_set_w_PI_cross_over(w) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_theta_e_offset", argv[1])) {
        double theta = strtod(argv[2], NULL);
        if (task_wolfpack_set_theta_e_offset(theta) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }




    return CMD_INVALID_ARGUMENTS;
}

#endif // APP_WOLFPACK
