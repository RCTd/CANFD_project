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

/*******************************************************************************
 * Variables
 ******************************************************************************/
flexcan_handle_t flexcanHandle;
flexcan_fd_frame_t rxFrame;
flexcan_fd_frame_t txFrame;
flexcan_mb_transfer_t txXfer; /* Transfer structure for Non-Blocking API */
volatile bool txComplete = false;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void FLEXCAN_PHY_Config_Safe(void);
static void UpdatePotentiometerState(uint8_t percentage, pot_state_t *currentPotState);

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
	gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 1};

	BOARD_InitHardware();
	LOG_INFO("--- No-Hang CAN FD Receiver ---\r\n");

	/* Initialize On-board RGB Pins */
	GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, &led_config);
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, &led_config);
	GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, &led_config);

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

static void UpdatePotentiometerState(uint8_t percentage, pot_state_t *currentPotState)
{
    /* 1. Determine the State based on input */

    GPIO_PinWrite(RED_PORT, RED_PIN, 0U);
    GPIO_PinWrite(YEL_PORT, YEL_PIN, 0U);
    GPIO_PinWrite(GRN_PORT, GRN_PIN, 0U);

    if (percentage <= 10)
	{
		*currentPotState = POT_STATE_ZERO;
	}
    else if (percentage > 10 && percentage <= 33)
    {
        *currentPotState = POT_STATE_LOW;
    }
    else if (percentage > 33 && percentage <= 66)
    {
        *currentPotState = POT_STATE_NORMAL;
    }
    else
    {
        *currentPotState = POT_STATE_CRITICAL;
    }

    PRINTF("   [Potentiometer] Value: %d%% -> ", percentage);

    /* 2. Execute Action based on State */
    switch (*currentPotState)
    {
    	case POT_STATE_ZERO:
            PRINTF("State: ZERO \r\n");
            GPIO_PinWrite(RED_PORT, RED_PIN, 0U);
            GPIO_PinWrite(YEL_PORT, YEL_PIN, 0U);
            GPIO_PinWrite(GRN_PORT, GRN_PIN, 0U);
			break;

        case POT_STATE_LOW:
            PRINTF("State: LOW \r\n");
            GPIO_PinWrite(GRN_PORT, GRN_PIN, 1U);
            break;

        case POT_STATE_NORMAL:
            PRINTF("State: NORMAL\r\n");
            GPIO_PinWrite(GRN_PORT, GRN_PIN, 1U);
            GPIO_PinWrite(YEL_PORT, YEL_PIN, 1U);
            break;

        case POT_STATE_CRITICAL:
            PRINTF("State: CRITICAL \r\n");
            GPIO_PinWrite(GRN_PORT, GRN_PIN, 1U);
            GPIO_PinWrite(YEL_PORT, YEL_PIN, 1U);
            GPIO_PinWrite(RED_PORT, RED_PIN, 1U);
            break;

        default:
            PRINTF("State: Unknown\r\n");
            break;
    }
}


static void UpdateTempState(uint8_t temperature)
{
    /* 1. Determine the State based on input */
	pot_state_t currentTmpState = POT_STATE_LOW;
	GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 1U);
	GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 1U);
	GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 1U);

    if (temperature <= 10)
	{
		currentTmpState = POT_STATE_ZERO;
	}
    else if (temperature > 10 && temperature <= 33)
    {
        currentTmpState = POT_STATE_LOW;
    }
    else if (temperature > 33 && temperature <= 37)
    {
        currentTmpState = POT_STATE_NORMAL;
    }
    else
    {
        currentTmpState = POT_STATE_CRITICAL;
    }

    PRINTF("   [Temperature] Value: %d%% -> ", temperature);

    /* 2. Execute Action based on State */
    switch (currentTmpState)
    {
    	case POT_STATE_ZERO:
            PRINTF("State: ZERO \r\n");
            GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 1U);
            GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 1U);
            GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 1U);
			break;

        case POT_STATE_LOW:
            PRINTF("State: LOW \r\n");
            GPIO_PinWrite(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_PIN, 0U);
            break;

        case POT_STATE_NORMAL:
            PRINTF("State: NORMAL\r\n");
            GPIO_PinWrite(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_PIN, 0U);
            break;

        case POT_STATE_CRITICAL:
            PRINTF("State: CRITICAL \r\n");
            GPIO_PinWrite(BOARD_LED_RED_GPIO, BOARD_LED_RED_PIN, 0U);
            break;

        default:
            PRINTF("State: Unknown\r\n");
            break;
    }
}


int main(void)
{
	pot_state_t potState = POT_STATE_LOW;

	Init_Peripherals();

    LOG_INFO("Waiting for ID 0x%X...\r\n", RX_MSG_ID);

    while (1)
    {
        /* Polling check using Flags (Non-blocking) */
        /* Use 1ULL to ensure 64-bit shift safety */
        if (FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, (1ULL << RX_MESSAGE_BUFFER_NUM)))
        {
            FLEXCAN_ClearMbStatusFlags(EXAMPLE_CAN, (1ULL << RX_MESSAGE_BUFFER_NUM));

            status_t status = FLEXCAN_ReadFDRxMb(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &rxFrame);

            if (status == kStatus_Success)
            {
            	uint8_t extractedByte = (uint8_t)((rxFrame.dataWord[0] >> 24) & 0xFF);

                LOG_INFO("RX ID: 0x%X [Data: 0x%02X]\r\n",
                         rxFrame.id >> CAN_ID_STD_SHIFT,
                         rxFrame.dataWord[0]);
                uint8_t source_id = rxFrame.dataWord[0] >> 16;
                switch(source_id){

                case MASTER_DHT:
                	UpdateTempState(extractedByte);
                	break;
                case MASTER_POT:
					UpdatePotentiometerState(extractedByte, &potState);
                	break;
                default:
                	LOG_INFO("Source ID unknown\r\n");
                	break;
                }
            }
        }
    }
}
