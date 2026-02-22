
#ifdef APP_WOLFPACK

#include "app_wolfpack.h"
#include "drv/gp3io_mux.h"
#include "drv/pwm.h"
#include "drv/encoder.h"
#include "drv/timing_manager.h"
#include "cmd/cmd_wolfpack.h"
#include "task_wolfpack.h"

void app_wolfpack_init(void)
{
    cmd_wolfpack_register();

    // Set GPIO ports
    gp3io_mux_set_device(GP3IO_MUX_1_BASE_ADDR, GP3IO_MUX_DEVICE1);
    // gp3io_mux_set_device(GP3IO_MUX_2_BASE_ADDR, 1);
    // gp3io_mux_set_device(GP3IO_MUX_3_BASE_ADDR, 1);
    // gp3io_mux_set_device(GP3IO_MUX_4_BASE_ADDR, 1);

    // Enable sensors in timing manager
    timing_manager_enable_sensor(AMDS_1);
    // timing_manager_enable_sensor(AMDS_2);
    // timing_manager_enable_sensor(AMDS_3);
    // timing_manager_enable_sensor(AMDS_4);
    timing_manager_enable_sensor(ENCODER);
    timing_manager_enable_sensor(ADC);

    encoder_set_pulses_per_rev(20000);

    pwm_set_deadtime_ns(300);
	pwm_set_switching_freq(TASK_WOLFPACK_UPDATES_PER_SEC);
	timing_manager_set_ratio(1);
	pwm_enable_hw(1);

}

#endif // APP_WOLFPACK


