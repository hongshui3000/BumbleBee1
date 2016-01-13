#ifndef PLATFORM_CONF_H
#define PLATFORM_CONF_H

#define PLATFORM_CLOCK (20000000)

/* [Raven] dbg_fwsim_log */
#define DBG_FWSIM_LOG 						0
#define FAKE_UART_ADDRESS 					0x10000000 // BB3 FW sim

/* [Raven] cm4_dsp_test */
#define TEST_CM4_DSP 						0
#define TEST_CM4_DSP_LITE 					1

/* [Raven] gdma_i2s_fw_sim */
#define TEST_GDMA_I2S_FW_SIM_WITH_CLARK 	0

/* CM4 instruction cycle test */
#define TEST_CM4_SPEED                      0
#define TEST_CM4_SPEED_WITH_GPIO            0

#define TEST_FREERTOS                       0

#define LOWER_STACK_EN                      0

#define UPPER_STACK_EN                      0
#define UPPER_STACK_TEST_EN                 0


#define TODO while(1) ;


#endif // PLATFORM_CONF_H
