/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "app.h"

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

    /* Enable clocks for GPIOs */
    CLOCK_EnableClock(kCLOCK_Gpio0); /* For Switch */
    CLOCK_EnableClock(kCLOCK_Gpio3); /* For TJA1152 STB */

    BOARD_InitPins();
    BOARD_BootClockPLL100M();
    BOARD_InitDebugConsole();
}
