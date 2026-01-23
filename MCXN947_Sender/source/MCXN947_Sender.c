/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_ctimer.h"
#include "fsl_common.h"
#include "board.h"
#include "app.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool g_reqSendButton3Msg = false;
volatile bool g_reqSendButton2Msg = false;
volatile bool g_reqSendTimerMsg = false;

/* FlexCAN Handle for Non-Blocking Transfers */
flexcan_handle_t flexcanHandle;
flexcan_fd_frame_t txFrame;
flexcan_mb_transfer_t txXfer; /* Transfer structure required for Non-Blocking API */
volatile bool txComplete = false;
volatile bool txError = false;

static ctimer_match_config_t matchConfig0;

/*******************************************************************************
 * Callback Function
 ******************************************************************************/
/* FIXED: result parameter changed to uint64_t to match driver signature */
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint64_t result, void *userData)
{
    if (status == kStatus_FLEXCAN_TxIdle)
    {
        txComplete = true;
    }
    /* FIXED: Removed kStatus_FLEXCAN_BusOff as it was undeclared.
       Bus Off usually comes as kStatus_FLEXCAN_ErrorStatus with flags in 'result' */
    else if (status == kStatus_FLEXCAN_ErrorStatus)
    {
        txError = true;
    }
}

/*******************************************************************************
 * Interrupt Handlers
 ******************************************************************************/
void BOARD_SW3_IRQ_HANDLER(void)
{
    uint32_t interruptFlags = GPIO_GpioGetInterruptFlags(GPIO0);

    /* SW3 (P0_6) */
    if ((interruptFlags & (1U << BOARD_SW3_GPIO_PIN)) != 0)
    {
        GPIO_GpioClearInterruptFlags(BOARD_SW3_GPIO, 1U << BOARD_SW3_GPIO_PIN);
        g_reqSendButton3Msg = true;

        /* Reset Timer */
        CTIMER_StopTimer(CTIMER);
        CTIMER_Reset(CTIMER);
        CTIMER_StartTimer(CTIMER);
    }

    /* SW2 (P0_23) */
    if ((interruptFlags & (1U << BOARD_SW2_GPIO_PIN)) != 0)
    {
        GPIO_GpioClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);
        g_reqSendButton2Msg = true;
    }
    SDK_ISR_EXIT_BARRIER;
}

void ctimer_match0_callback(uint32_t flags)
{
    CTIMER_StopTimer(CTIMER);
    g_reqSendTimerMsg = true;
}

ctimer_callback_t ctimer_callback_table[] = {
    ctimer_match0_callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

static void SendCanMessageNonBlocking(uint32_t id, uint8_t dataByte)
{
    /* If a previous transfer is still stuck (rare with callback logic), don't overwrite */
    if (FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, 1ULL << TX_MESSAGE_BUFFER_NUM))
    {
        LOG_INFO("Busy. Skipping send.\r\n");
        return;
    }

    txFrame.id     = FLEXCAN_ID_STD(id);
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.edl    = 1U;
    txFrame.brs    = 1U;
    txFrame.length = 1U;
    txFrame.dataWord[0] = CAN_WORD_DATA_BYTE_0(dataByte);

    /* Prepare Transfer Structure */
    txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;
    txXfer.framefd = &txFrame;

    txComplete = false;
    txError = false;

    LOG_INFO("Sending ID: 0x%X... \r\n", id);

    /* Use Non-Blocking API */
    status_t status = FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

    if (status != kStatus_Success)
    {
        LOG_INFO("Fail to queue (Stat: %d).\r\n", status);
    }
}

static void FLEXCAN_PHY_Config_Safe(void)
{
    /* Setup TX MB */
    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);

    /* Wake Transceiver */
    gpio_pin_config_t stb_config = {kGPIO_DigitalOutput, 1};
    GPIO_PinInit(EXAMPLE_STB_RGPIO, EXAMPLE_STB_RGPIO_PIN, &stb_config);
    GPIO_PortSet(EXAMPLE_STB_RGPIO, 1u << EXAMPLE_STB_RGPIO_PIN);

    /* Prepare Handshake Frame */
    txFrame.id = FLEXCAN_ID_STD(0x555);
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.length = 0U;
    txFrame.edl = 0U;
    txFrame.brs = 0U;

    txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;
    txXfer.framefd = &txFrame;

    txComplete = false;

    /* SEND WITH TIMEOUT PROTECTION */
    FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

    /* Wait max 100ms for ACK */
    volatile uint32_t timeout = 1000000;
    while (!txComplete && timeout > 0)
    {
        timeout--;
    }

    if (timeout == 0)
    {
        LOG_INFO("PHY Config Timeout (No Bus ACK). Aborting...\r\n");
        FLEXCAN_TransferFDAbortSend(EXAMPLE_CAN, &flexcanHandle, TX_MESSAGE_BUFFER_NUM);
    }
    else
    {
        LOG_INFO("PHY Config ACK Received.\r\n");
    }

    /* Set STB Low (Normal Mode) */
    GPIO_PortClear(EXAMPLE_STB_RGPIO, 1u << EXAMPLE_STB_RGPIO_PIN);
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    flexcan_config_t flexcanConfig;
    ctimer_config_t ctimerConfig;
    gpio_pin_config_t sw_config = { kGPIO_DigitalInput, 0 };

    BOARD_InitHardware();
    LOG_INFO("--- No-Hang CAN FD Sender ---\r\n");

    /* GPIO Init */
    GPIO_SetPinInterruptConfig(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, kGPIO_InterruptFallingEdge);
    GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);
    GPIO_SetPinInterruptConfig(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, kGPIO_InterruptFallingEdge);
    GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);
    EnableIRQ(BOARD_SW3_IRQ);

    /* CTimer Init */
    CTIMER_GetDefaultConfig(&ctimerConfig);
    CTIMER_Init(CTIMER, &ctimerConfig);
    matchConfig0.enableCounterReset = true;
    matchConfig0.enableCounterStop = false;
    matchConfig0.matchValue = CTIMER_CLK_FREQ * 1;
    matchConfig0.outControl = kCTIMER_Output_NoAction;
    matchConfig0.outPinInitState = false;
    matchConfig0.enableInterrupt = true;
    CTIMER_RegisterCallBack(CTIMER, &ctimer_callback_table[0], kCTIMER_MultipleCallback);
    CTIMER_SetupMatch(CTIMER, CTIMER_MAT0_OUT, &matchConfig0);
    EnableIRQ(CTIMER_IRQ_ID);

    /* FlexCAN Init */
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

    /* Create the Handle for Interrupt-driven transfers */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* Configure PHY safely */
    FLEXCAN_PHY_Config_Safe();

    /* Set up Tx MB */
    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
    /* FIXED: Removed txFrame.mbIdx assignment */

    LOG_INFO("System Ready.\r\n");

    while (1)
    {
        if (g_reqSendButton3Msg)
        {
            g_reqSendButton3Msg = false;
            SendCanMessageNonBlocking(TX_MSG_ID_TIMER, 0x33);
        }

        if (g_reqSendButton2Msg)
        {
            g_reqSendButton2Msg = false;
            SendCanMessageNonBlocking(TX_MSG_ID_TIMER, 0x22);
        }

        if (g_reqSendTimerMsg)
        {
            g_reqSendTimerMsg = false;
            SendCanMessageNonBlocking(TX_MSG_ID_TIMER, 0xBB);
        }
    }
}
