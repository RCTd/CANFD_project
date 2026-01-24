/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "app.h"

#include "fsl_vref.h"
#include "fsl_spc.h"
#include "fsl_common.h"

void BOARD_InitHardware(void)
{
    /* attach FRO 12M to FLEXCOMM4 (debug console) */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* attach PLL1Clk0 to FLEXCAN0 */
    CLOCK_SetClkDiv(kCLOCK_DivPLL1Clk0, 2U);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcan0Clk, 1U);
    CLOCK_AttachClk(kPLL1_CLK0_to_FLEXCAN0);

    /* Use FRO HF clock for CTIMER4 */
    CLOCK_SetClkDiv(kCLOCK_DivCtimer4Clk, 1u);
    CLOCK_AttachClk(kFRO_HF_to_CTIMER4);

    /* attach FRO HF to SCT */
	CLOCK_SetClkDiv(kCLOCK_DivSctClk, 1u);
	CLOCK_AttachClk(kFRO_HF_to_SCT);

    /* Enable clocks for GPIOs */
    CLOCK_EnableClock(kCLOCK_Gpio0); /* For Switch */
    CLOCK_EnableClock(kCLOCK_Gpio3); /* For TJA1152 STB */

    /* Enable ADC */

    vref_config_t vrefConfig;

    /* attach FRO 12M to FLEXCOMM4 (debug console) */
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1U);
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* attach FRO HF to ADC0 */
    CLOCK_SetClkDiv(kCLOCK_DivAdc0Clk, 1U);
    CLOCK_AttachClk(kFRO_HF_to_ADC0);

    /* enable VREF */
    SPC_EnableActiveModeAnalogModules(DEMO_SPC_BASE, kSPC_controlVref);

    VREF_GetDefaultConfig(&vrefConfig);
    vrefConfig.bufferMode = kVREF_ModeBandgapOnly;
    /* Initialize VREF module, the VREF module is only used to supply the bias current for LPADC. */
    VREF_Init(DEMO_VREF_BASE, &vrefConfig);

    BOARD_InitPins();
    BOARD_BootClockPLL100M();
    BOARD_InitDebugConsole();
}
