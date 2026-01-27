	/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

void BOARD_InitPins(void)
{
    /* Enable Clock for Port0, Port1, Port3 */
    CLOCK_EnableClock(kCLOCK_Port0);
    CLOCK_EnableClock(kCLOCK_Port1);
    CLOCK_EnableClock(kCLOCK_Port2);
    CLOCK_EnableClock(kCLOCK_Port3);

    /* --- UART (Debug Console) --- */
    /* PORT1_8 is FC4_P0 (TX) */
    PORT_SetPinMux(PORT1, 8U, kPORT_MuxAlt2);
    PORT_SetPinConfig(PORT1, 8U, &(const port_pin_config_t){
        kPORT_PullDisable, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
        kPORT_MuxAlt2, kPORT_InputBufferEnable, kPORT_InputNormal, kPORT_UnlockRegister
    });

    /* PORT1_9 is FC4_P1 (RX) */
    PORT_SetPinMux(PORT1, 9U, kPORT_MuxAlt2);
    PORT_SetPinConfig(PORT1, 9U, &(const port_pin_config_t){
        kPORT_PullDisable, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
        kPORT_MuxAlt2, kPORT_InputBufferEnable, kPORT_InputNormal, kPORT_UnlockRegister
    });

	/* --- SW2 Configuration (PORT0 Pin 23) --- */
	const port_pin_config_t sw2_config = {
		kPORT_PullUp,               /* Internal pull-up resistor is enabled */
		kPORT_LowPullResistor,
		kPORT_FastSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_OpenDrainDisable,
		kPORT_LowDriveStrength,
		kPORT_MuxAlt0,              /* Pin is configured as PIO0_23 (GPIO) */
		kPORT_InputBufferEnable,    /* Digital input enabled */
		kPORT_InputNormal,
		kPORT_UnlockRegister
	};
	/* Configure PORT0 Pin 23 as GPIO (SW2) */
	PORT_SetPinConfig(PORT0, 23U, &sw2_config);

	/* --- LED Configuration (Red LED on P0_10) --- */
	const port_pin_config_t led_config = {
		kPORT_PullDisable,
		kPORT_LowPullResistor,
		kPORT_FastSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_OpenDrainDisable,
		kPORT_LowDriveStrength,
		kPORT_MuxAlt0, /* GPIO Mode */
		kPORT_InputBufferEnable,
		kPORT_InputNormal,
		kPORT_UnlockRegister
	};
	PORT_SetPinConfig(PORT0, 10U, &led_config);

	const port_pin_config_t out_config = {
		kPORT_PullDisable, kPORT_LowPullResistor, kPORT_FastSlewRate,
		kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
		kPORT_MuxAlt0, kPORT_InputBufferEnable, kPORT_InputNormal, kPORT_UnlockRegister
	};

	/* Configurare pini Semafor pe J2 (8, 10, 12) */
	PORT_SetPinConfig(PORT0, 24U, &out_config); /* Rosu */
	PORT_SetPinConfig(PORT0, 26U, &out_config); /* Galben */
	PORT_SetPinConfig(PORT0, 25U, &out_config); /* Verde */

	/* Ensure CAN pins (P1_10, P1_11) and STB pin (P3_13) are also configured here */

    /* --- CAN0 Pins --- */
    /* PORT1_10 as CAN0_TXD */
    PORT_SetPinMux(PORT1, 10U, kPORT_MuxAlt11);
    PORT_SetPinConfig(PORT1, 10U, &(const port_pin_config_t){
        kPORT_PullDisable, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
        kPORT_MuxAlt11, kPORT_InputBufferEnable, kPORT_InputNormal, kPORT_UnlockRegister
    });

    /* PORT1_11 as CAN0_RXD */
    PORT_SetPinMux(PORT1, 11U, kPORT_MuxAlt11);
    PORT_SetPinConfig(PORT1, 11U, &(const port_pin_config_t){
        kPORT_PullDisable, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
        kPORT_MuxAlt11, kPORT_InputBufferEnable, kPORT_InputNormal, kPORT_UnlockRegister
    });

    /* --- GPIO Switch (SW3) --- */
    /* PORT0_6 as GPIO */
    PORT_SetPinMux(PORT0, 6U, kPORT_MuxAlt0);
    PORT_SetPinConfig(PORT0, 6U, &(const port_pin_config_t){
        kPORT_PullUp, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
        kPORT_MuxAlt0, kPORT_InputBufferEnable, kPORT_InputNormal, kPORT_UnlockRegister
    });

    /* --- TJA1152 STB Pin (Required for CAN Transceiver on EVK) --- */
    /* PORT3_13 as GPIO */
    PORT_SetPinMux(PORT3, 13U, kPORT_MuxAlt0);
    /* Basic digital output config */
    const port_pin_config_t stb_pin_config = {
        kPORT_PullDisable, kPORT_LowPullResistor, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable, kPORT_LowDriveStrength,
        kPORT_MuxAlt0, kPORT_InputBufferEnable, kPORT_InputNormal, kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORT3, 13U, &stb_pin_config);


    const port_pin_config_t port2_2_pinH3_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as PWM1_A2 */
                                                    kPORT_MuxAlt5,
                                                    /* Digital input enabled */
                                                    kPORT_InputBufferEnable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT2_2 (pin H3) is configured as PWM1_A2 */
    PORT_SetPinConfig(PORT2, 2U, &port2_2_pinH3_config);


    const port_pin_config_t port2_4_pinK3_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as PWM1_A1 */
                                                    kPORT_MuxAlt5,
                                                    /* Digital input enabled */
                                                    kPORT_InputBufferEnable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT2_4 (pin K3) is configured as PWM1_A1 */
    PORT_SetPinConfig(PORT2, 4U, &port2_4_pinK3_config);

    const port_pin_config_t port2_6_pinK2_config = {/* Internal pull-up/down resistor is disabled */
                                                    kPORT_PullDisable,
                                                    /* Low internal pull resistor value is selected. */
                                                    kPORT_LowPullResistor,
                                                    /* Fast slew rate is configured */
                                                    kPORT_FastSlewRate,
                                                    /* Passive input filter is disabled */
                                                    kPORT_PassiveFilterDisable,
                                                    /* Open drain output is disabled */
                                                    kPORT_OpenDrainDisable,
                                                    /* Low drive strength is configured */
                                                    kPORT_LowDriveStrength,
                                                    /* Pin is configured as PWM1_A0 */
                                                    kPORT_MuxAlt5,
                                                    /* Digital input enabled */
                                                    kPORT_InputBufferEnable,
                                                    /* Digital input is not inverted */
                                                    kPORT_InputNormal,
                                                    /* Pin Control Register fields [15:0] are not locked */
                                                    kPORT_UnlockRegister};
    /* PORT2_6 (pin K2) is configured as PWM1_A0 */
    PORT_SetPinConfig(PORT2, 6U, &port2_6_pinK2_config);

    const port_pin_config_t port2_7_pinL2_config = {/* Internal pull-up/down resistor is disabled */
                                                     kPORT_PullDisable,
                                                     /* Low internal pull resistor value is selected. */
                                                     kPORT_LowPullResistor,
                                                     /* Fast slew rate is configured */
                                                     kPORT_FastSlewRate,
                                                     /* Passive input filter is disabled */
                                                     kPORT_PassiveFilterDisable,
                                                     /* Open drain output is disabled */
                                                     kPORT_OpenDrainDisable,
                                                     /* Low drive strength is configured */
                                                     kPORT_LowDriveStrength,
                                                     /* Pin is configured as PWM1_B0 */
                                                     kPORT_MuxAlt5,
                                                     /* Digital input enabled */
                                                     kPORT_InputBufferEnable,
                                                     /* Digital input is not inverted */
                                                     kPORT_InputNormal,
                                                     /* Pin Control Register fields [15:0] are not locked */
                                                     kPORT_UnlockRegister};
     /* PORT2_7 (pin L2) is configured as PWM1_B0 */
     PORT_SetPinConfig(PORT2, 7U, &port2_7_pinL2_config);

         const port_pin_config_t servo_pwm_config = {
             kPORT_PullDisable,
             kPORT_LowPullResistor,
             kPORT_FastSlewRate,
             kPORT_PassiveFilterDisable,
             kPORT_OpenDrainDisable,
             kPORT_LowDriveStrength,
             kPORT_MuxAlt5,
             kPORT_InputBufferEnable,
             kPORT_InputNormal,
             kPORT_UnlockRegister
         };
         /* PORT1_8 configured as SCT0_OUT2 */
         PORT_SetPinConfig(PORT1, 8U, &servo_pwm_config);
}
