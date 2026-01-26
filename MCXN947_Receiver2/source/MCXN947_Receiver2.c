/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_common.h"
#include "board.h"
#include "app.h"
#include "fsl_sctimer.h"  // Necesar pentru SCTIMER_Init, UpdateDutycycle etc.
#include "fsl_clock.h"    // Necesar pentru CLOCK_GetFreq
#include <string.h>
// Pin Configuration for FRDM-MCXN947
// We are using the pin labeled "D2" on the board (which is P0_29)
#define SERVO_GPIO_PORT   GPIO0
#define SERVO_GPIO_PIN    29U


#define BOARD_SCTIMER      SCT0
#define BOARD_SCT_OUT      kSCTIMER_Out_2
#define SCTIMER_CLK_FREQ   CLOCK_GetFreq(kCLOCK_BusClk)


// Servo Timing Constants (in Microseconds)
#define SERVO_PERIOD_US   20000U   // 20ms period (50Hz)
#define MIN_PULSE_US      500U    // 0 degrees
#define MAX_PULSE_US      2500U    // 180 degrees

/*******************************************************************************
 * SCTimer Definitions
 ******************************************************************************/
#define SCTIMER_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define DEMO_SCTIMER_OUT kSCTIMER_Out_2 /* Verifică maparea pentru P0_29 */

uint32_t sctimer_event;
uint32_t sctimer_event_number;

/* Global variables to manage state between Main and Interrupt */
volatile int g_current_angle = 0;
volatile int g_target_angle = 0;
volatile int g_speed_delay_ticks = 0; // Number of 20ms cycles to wait per degree
volatile int g_tick_counter = 0;

/* This function runs automatically every 20ms (End of PWM cycle) */
void SCT0_IRQHandler(void)
{
    // 1. Check if the interrupt was caused by our PWM Event (Event 0 is typically the period)
    if (SCTIMER_GetStatusFlags(BOARD_SCTIMER) & (1 << kSCTIMER_Event0Flag))
    {
        // Clear the flag so the interrupt doesn't fire again immediately
        SCTIMER_ClearStatusFlags(BOARD_SCTIMER, (1 << kSCTIMER_Event0Flag));

        // 2. Check if we need to move
        if (g_current_angle != g_target_angle)
        {
            g_tick_counter++;

            // Only move if we have waited enough cycles (Speed Control)
            if (g_tick_counter >= g_speed_delay_ticks)
            {
                // Determine direction (+1 or -1)
                double step = (g_target_angle > g_current_angle) ? 1 : -1;
                g_current_angle += step;

                // --- Perform the Hardware Update (Same logic as before) ---
                uint32_t pulse_us = 500U + ((uint32_t)g_current_angle * (2500U - 500U) / 180U);
                uint32_t dutyCyclePercent = (pulse_us * 100U) / 20000U;
                SCTIMER_UpdatePwmDutycycle(BOARD_SCTIMER, BOARD_SCT_OUT, dutyCyclePercent, sctimer_event_number);
                // --------------------------------------------------------

                // Reset counter for the next step
                g_tick_counter = 0;
            }
        }
    }

    // Allow SDK to handle any other flags if necessary
    SDK_ISR_EXIT_BARRIER;
}





void init_sctimer_pwm(void) {
    sctimer_config_t sctimerConfig;
    sctimer_pwm_signal_param_t pwmParam;

    /* 1. Clock Setup */
    CLOCK_AttachClk(kPLL1_CLK0_to_SCT);
    CLOCK_EnableClock(kCLOCK_Sct);
    RESET_PeripheralReset(kSCT_RST_SHIFT_RSTn);

    /* 2. Frequency Setup - Using SystemCoreClock to ensure it is non-zero */
    uint32_t sctimerClock = SystemCoreClock;

    /* 3. Force Unified Mode (32-bit) */
    SCTIMER_GetDefaultConfig(&sctimerConfig);
    sctimerConfig.enableCounterUnify = true; // Set to True to satisfy the new assert

    if (SCTIMER_Init(BOARD_SCTIMER, &sctimerConfig) != kStatus_Success) {
        PRINTF("SCTimer Init Failed\r\n");
    }

    /* 4. PWM Parameter Setup */
    pwmParam.output           = BOARD_SCT_OUT;
    pwmParam.level            = kSCTIMER_HighTrue;
    pwmParam.dutyCyclePercent = 5; // 5% of 20ms is 1ms

    /* 5. Setup PWM */
    /* When Unify is true, the driver configures the 32-bit counter */

    SCTIMER_EnableInterrupts(BOARD_SCTIMER, (1 << kSCTIMER_Event0Flag));
    EnableIRQ(SCT0_IRQn);

    if (SCTIMER_SetupPwm(BOARD_SCTIMER, &pwmParam, kSCTIMER_EdgeAlignedPwm,
                         50U, sctimerClock, &sctimer_event_number) != kStatus_Success) {
        PRINTF("SCTimer PWM Setup Failed!\r\n");
    }

    /* 6. Start the UNIFIED counter */
    /* Changed from kSCTIMER_Counter_L to kSCTIMER_Counter_U */
    SCTIMER_StartTimer(BOARD_SCTIMER, kSCTIMER_Counter_U);
}
/*******************************************************************************
 * Variables
 ******************************************************************************/
flexcan_handle_t flexcanHandle;
flexcan_fd_frame_t rxFrame;
flexcan_fd_frame_t txFrame;
flexcan_mb_transfer_t txXfer; /* Transfer structure for Non-Blocking API */
volatile bool txComplete = false;

int angle_change = 100;
int angle = 180;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void FLEXCAN_PHY_Config_Safe(void);
static void SetNextTrafficLight(traffic_state_t *currentState);

/*******************************************************************************
 * Code
 ******************************************************************************/


/* FIXED: Callback result type changed to uint64_t to match driver signature */
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint64_t result, void *userData)
{
    if (status == kStatus_FLEXCAN_TxIdle)
    {
        txComplete = true;
    }
    /* We don't need to handle RX here because we are polling in main for simplicity,
       but we NEED this callback for the PHY Config Non-Blocking send. */
}

/* Safe PHY Configuration with Timeout */
static void FLEXCAN_PHY_Config_Safe(void)
{
    /* Use MB0 for temporary transmission */
    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, 0, true);

    gpio_pin_config_t stb_config = {kGPIO_DigitalOutput, 1};
    GPIO_PinInit(EXAMPLE_STB_RGPIO, EXAMPLE_STB_RGPIO_PIN, &stb_config);
    GPIO_PortSet(EXAMPLE_STB_RGPIO, 1u << EXAMPLE_STB_RGPIO_PIN);

    gpio_pin_config_t out_cfg = { kGPIO_DigitalOutput, 0 };

	GPIO_PinInit(RED_PORT, RED_PIN, &out_cfg);
	GPIO_PinInit(YEL_PORT, YEL_PIN, &out_cfg);
	GPIO_PinInit(GRN_PORT, GRN_PIN, &out_cfg);

    /* Prepare Handshake Frame */
    txFrame.id = FLEXCAN_ID_STD(0x555);
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.length = 0U;
    txFrame.edl = 0U;
    txFrame.brs = 0U;

    /* FIXED: Assign mbIdx to the transfer structure, not the frame */
    txXfer.mbIdx = 0;
    txXfer.framefd = &txFrame;

    txComplete = false;

    /* Start Non-Blocking Send using the Transfer structure */
    FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

    /* Wait with Timeout */
    volatile uint32_t timeout = 1000000;
    while (!txComplete && timeout > 0)
    {
        timeout--;
    }

    if (timeout == 0)
    {
        LOG_INFO("PHY Config Timeout (No Bus ACK). Aborting...\r\n");
        FLEXCAN_TransferFDAbortSend(EXAMPLE_CAN, &flexcanHandle, 0);
    }
    else
    {
        LOG_INFO("PHY Config ACK Received.\r\n");
    }

    GPIO_PortClear(EXAMPLE_STB_RGPIO, 1u << EXAMPLE_STB_RGPIO_PIN);
}

static void Init_Peripherals(void){
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;
	BOARD_InitHardware();
	init_sctimer_pwm();

	gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 1};


	LOG_INFO("--- No-Hang CAN FD Receiver ---\r\n");

	GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);

	FLEXCAN_GetDefaultConfig(&flexcanConfig);
//    flexcanConfig.bitRate = 500000U;
	flexcanConfig.bitRateFD = 2000000U;

#if defined(EXAMPLE_CAN_CLK_SOURCE)
	flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
#endif

	flexcan_timing_config_t timing_config;
	memset(&timing_config, 0, sizeof(flexcan_timing_config_t));
	if (FLEXCAN_FDCalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.bitRate, flexcanConfig.bitRateFD,
												EXAMPLE_CAN_CLK_FREQ, &timing_config))
	{
		memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
	}

	FLEXCAN_FDInit(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ, BYTES_IN_MB, true);

	/* Create Handle for Interrupt Support */
	FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

	/* Safe PHY Config */
	FLEXCAN_PHY_Config_Safe();

	/* Setup Receive Message Buffer (MB9) */
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
	mbConfig.type   = kFLEXCAN_FrameTypeData;
	mbConfig.id     = FLEXCAN_ID_STD(RX_MSG_ID);

	FLEXCAN_SetFDRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
	FLEXCAN_SetRxIndividualMask(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, FLEXCAN_RX_MB_STD_MASK(0x7FF, 0, 0));
}





void set_servo_async(int angle, int speed_delay_ms)
{
    // Clamp constraints
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Calculate how many 20ms cycles we need to wait per degree
    // Example: If speed_delay_ms is 60ms, we wait 3 cycles (60 / 20) per degree.
    // Minimum is 0 (move every cycle, which is max speed of ~50 degrees/sec).
    int ticks = speed_delay_ms / 20;

    // Atomic update of settings
    DisableIRQ(SCT0_IRQn); // Pause IRQ briefly to prevent data corruption
    g_target_angle = angle;
    g_speed_delay_ticks = ticks;
    g_tick_counter = 0;    // Reset counter to start moving immediately
    EnableIRQ(SCT0_IRQn);  // Resume IRQ
}

int main(void)
{
	// În Init_Peripherals() sau la începutul main(), adaugă:
	//SysTick_Config(SystemCoreClock / 1000); // Configurare pentru 1ms
    /* Hardware Initialization */
    Init_Peripherals();

    /* Configurare SysTick pentru 1ms */
    //SysTick_Config(SystemCoreClock / 1000);

    LOG_INFO("System Ready. Waiting for CAN messages...\r\n");
    set_servo_async(angle, angle_change);

    while (1) {
        /* --- PARTEA 1: RECEPȚIE CAN (Non-blocking) --- */
        if (FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, (1ULL << RX_MESSAGE_BUFFER_NUM)))
        {
            FLEXCAN_ClearMbStatusFlags(EXAMPLE_CAN, (1ULL << RX_MESSAGE_BUFFER_NUM));
            status_t status = FLEXCAN_ReadFDRxMb(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &rxFrame);

            if (status == kStatus_Success)
            {
                uint32_t canData = rxFrame.dataWord[0];
                int data_read = (canData >> 24) & 255;
                int sender_id = (canData >> 16) & 3;

                if (sender_id & 1) { // Potențiometru
                    angle = data_read * 1.8;
                }
                else if (sender_id & 2) { // Senzor
                    angle_change = 100 * (100-data_read) / 60;
                }
                LOG_INFO("CAN RX. ID:%d Data:%d -> Next Move Prep\r\n", sender_id, data_read);
                LOG_INFO("Angle: %d angle_change: %d\r\n", angle,angle_change);

                set_servo_async(angle, angle_change);

            }
        }


    }
}
