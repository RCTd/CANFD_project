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
 * Definitions & FIFO Settings
 ******************************************************************************/
#define FIFO_SIZE 16 /* Must be power of 2 */

typedef struct
{
    uint32_t id;
    uint8_t dataByte;
} can_msg_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* FIFO Variables */
volatile can_msg_t fifo_buffer[FIFO_SIZE];
volatile uint8_t fifo_head = 0;
volatile uint8_t fifo_tail = 0;

/* FlexCAN Handle and Variables */
flexcan_handle_t flexcanHandle;
flexcan_fd_frame_t txFrame;
flexcan_mb_transfer_t txXfer;
volatile bool txComplete = true;
volatile bool txError = false;

static ctimer_match_config_t matchConfig0;

/*******************************************************************************
 * FIFO Helper Functions
 ******************************************************************************/
bool FIFO_Push(uint32_t id, uint8_t data)
{
    uint8_t next_head = (fifo_head + 1) % FIFO_SIZE;

    if (next_head == fifo_tail)
    {
        return false; /* Buffer Full */
    }

    fifo_buffer[fifo_head].id = id;
    fifo_buffer[fifo_head].dataByte = data;
    fifo_head = next_head;
    return true;
}

bool FIFO_Pop(can_msg_t *msg)
{
    if (fifo_head == fifo_tail)
    {
        return false; /* Buffer Empty */
    }

    *msg = fifo_buffer[fifo_tail];
    fifo_tail = (fifo_tail + 1) % FIFO_SIZE;
    return true;
}

bool FIFO_IsEmpty(void)
{
    return (fifo_head == fifo_tail);
}

/*******************************************************************************
 * Callback Function
 ******************************************************************************/
static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint64_t result, void *userData)
{
    if (status == kStatus_FLEXCAN_TxIdle)
    {
        txComplete = true;
    }
    else if (status == kStatus_FLEXCAN_ErrorStatus)
    {
        txError = true;
        txComplete = true;
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

        CTIMER_StopTimer(CTIMER);
        CTIMER_Reset(CTIMER);
        CTIMER_StartTimer(CTIMER);

        FIFO_Push(TX_MSG_ID_TIMER, 0x33);
    }

    /* SW2 (P0_23) */
    if ((interruptFlags & (1U << BOARD_SW2_GPIO_PIN)) != 0)
    {
        GPIO_GpioClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);
        FIFO_Push(TX_MSG_ID_TIMER, 0x22);
    }
    SDK_ISR_EXIT_BARRIER;
}

void ctimer_match0_callback(uint32_t flags)
{
    CTIMER_StopTimer(CTIMER);
    FIFO_Push(TX_MSG_ID_TIMER, 0xBB);
}

ctimer_callback_t ctimer_callback_table[] = {
    ctimer_match0_callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

static void SendCanMessageNonBlocking(uint32_t id, uint8_t dataByte)
{
    txFrame.id     = FLEXCAN_ID_STD(id);
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.edl    = 1U;
    txFrame.brs    = 1U;
    txFrame.length = 1U;
    txFrame.dataWord[0] = CAN_WORD_DATA_BYTE_0(dataByte);

    txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;
    txXfer.framefd = &txFrame;

    txComplete = false;
    txError = false;

    LOG_INFO("Sending ID: 0x%X Data: 0x%02X \r\n", id, dataByte);

    status_t status = FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

    if (status != kStatus_Success)
    {
        LOG_INFO("Fail to queue (Stat: %d).\r\n", status);
        txComplete = true;
    }
}

static void FLEXCAN_PHY_Config_Safe(void)
{
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
    FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

    volatile uint32_t timeout = 1000000;
    while (!txComplete && timeout > 0)
    {
        timeout--;
    }

    if (timeout == 0)
    {
        LOG_INFO("PHY Config Timeout (No Bus ACK). Aborting...\r\n");
        FLEXCAN_TransferFDAbortSend(EXAMPLE_CAN, &flexcanHandle, TX_MESSAGE_BUFFER_NUM);
        txComplete = true;
    }
    else
    {
        LOG_INFO("PHY Config ACK Received.\r\n");
    }

    GPIO_PortClear(EXAMPLE_STB_RGPIO, 1u << EXAMPLE_STB_RGPIO_PIN);
}

/*******************************************************************************
 * Initialization Function
 ******************************************************************************/
static void Init_Peripherals(void)
{
    flexcan_config_t flexcanConfig;
    ctimer_config_t ctimerConfig;
    gpio_pin_config_t sw_config = { kGPIO_DigitalInput, 0 };
    flexcan_timing_config_t timing_config;

    /* 1. Board Init */
    BOARD_InitHardware();
    LOG_INFO("--- No-Hang CAN FD Sender (FIFO Enabled) ---\r\n");

    /* 2. GPIO Init */
    GPIO_SetPinInterruptConfig(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, kGPIO_InterruptFallingEdge);
    GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);
    GPIO_SetPinInterruptConfig(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, kGPIO_InterruptFallingEdge);
    GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);
    EnableIRQ(BOARD_SW3_IRQ);

    /* 3. CTimer Init */
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

    /* 4. FlexCAN Init */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);
    flexcanConfig.bitRateFD = 2000000U;
#if defined(EXAMPLE_CAN_CLK_SOURCE)
    flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
#endif

    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));
    if (FLEXCAN_FDCalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.bitRate, flexcanConfig.bitRateFD,
                                                EXAMPLE_CAN_CLK_FREQ, &timing_config))
    {
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }

    FLEXCAN_FDInit(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ, BYTES_IN_MB, true);
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* 5. FlexCAN PHY & MB Setup */
    FLEXCAN_PHY_Config_Safe();
    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);

    /* Ensure txComplete starts true for the main loop */
    txComplete = true;

    LOG_INFO("Initialization Complete. Waiting for events...\r\n");
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    /* Call the consolidated Init function */
    Init_Peripherals();

    can_msg_t nextMsg;

    while (1)
    {
        /* CRITICAL SECTION: Check FIFO logic atomically */
        __disable_irq();
        bool hasData = !FIFO_IsEmpty();
        __enable_irq();

        /* If we have data AND the previous transfer is done */
        if (hasData && txComplete)
        {
            /* Pop the message safely */
            __disable_irq();
            bool success = FIFO_Pop(&nextMsg);
            __enable_irq();

            if (success)
            {
                SendCanMessageNonBlocking(nextMsg.id, nextMsg.dataByte);
            }
        }
    }
}
