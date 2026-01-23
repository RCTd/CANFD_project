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

    const port_pin_config_t port1_22_pinL4_config = {/* Internal pull-up/down resistor is disabled */
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
													 /* Pin is configured as SCT0_OUT4 */
													 kPORT_MuxAlt5,
													 /* Digital input enabled */
													 kPORT_InputBufferEnable,
													 /* Digital input is not inverted */
													 kPORT_InputNormal,
													 /* Pin Control Register fields [15:0] are not locked */
													 kPORT_UnlockRegister};
	/* PORT1_22 (pin L4) is configured as SCT0_OUT4 */
	PORT_SetPinConfig(PORT1, 22U, &port1_22_pinL4_config);

	const port_pin_config_t port1_23_pinM4_config = {/* Internal pull-up/down resistor is disabled */
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
													 /* Pin is configured as SCT0_OUT5 */
													 kPORT_MuxAlt5,
													 /* Digital input enabled */
													 kPORT_InputBufferEnable,
													 /* Digital input is not inverted */
													 kPORT_InputNormal,
													 /* Pin Control Register fields [15:0] are not locked */
													 kPORT_UnlockRegister};
	/* PORT1_23 (pin M4) is configured as SCT0_OUT5 */
	PORT_SetPinConfig(PORT1, 23U, &port1_23_pinM4_config);

	const port_pin_config_t port1_6_pinD2_config = {/* Internal pull-up/down resistor is disabled */
														kPORT_PullUp,
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
														/* Pin is configured as FC4_P1 */
														kPORT_MuxAsGpio,
														/* Digital input enabled */
														kPORT_InputBufferEnable,
														/* Digital input is not inverted */
														kPORT_InputNormal,
														/* Pin Control Register fields [15:0] are not locked */
														kPORT_UnlockRegister};
		/* PORT1_6 (pin D2) is configured as asdsad */
		PORT_SetPinConfig(PORT1, 6U, &port1_6_pinD2_config);

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
}
