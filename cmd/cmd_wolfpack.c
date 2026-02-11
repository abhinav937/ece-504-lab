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
	{ "sm_idle", "Request SM Idle, only acknowledged from Running" },
	{ "sm_calibrate", "Request SM Calibrate, only acknowledged from Idle" },
	{ "sm_run", "Request SM Run, only acknowledged from Idle" },
	{ "sm_trip_clear", "Request SM Trip Clear, only acknowledged from Tripped" },
    { "set_manual_we", "Set frequency of voltage output (rad/s)" },
    { "set_manual_Vmag", "Set amplitude of voltage output (V)" },
	{ "set_VperHz_RPM", "Set the speed for Volt/Hz mode (RPM)" },
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

    if (argc == 3 && STREQ("set_manual_we", argv[1])) {
        double w_e = strtod(argv[2], NULL);

        if (task_wolfpack_set_manual_w_e(w_e) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_manual_Vmag", argv[1])) {
        double Vmag = strtod(argv[2], NULL);

        if (task_wolfpack_set_manual_V_mag(Vmag) != SUCCESS) {
            return CMD_FAILURE;
        }
        return CMD_SUCCESS;
    }

    if (argc == 3 && STREQ("set_VperHz_RPM", argv[1])) {
        double speed_ref = strtod(argv[2], NULL);
        task_wolfpack_set_Volt_per_Hz_speed(speed_ref);
        return CMD_SUCCESS;
    }

    return CMD_INVALID_ARGUMENTS;
}

#endif // APP_WOLFPACK
