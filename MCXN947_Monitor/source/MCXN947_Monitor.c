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
flexcan_mb_transfer_t txXfer;
volatile bool txComplete = false;

#define MASTER_POTENTIOMETER 1
#define MASTER_DHT 2
#define SLAVE_SERVO 0x322
#define SLAVE_LED 0x321

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void FLEXCAN_PHY_Config_Safe(void);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint64_t result, void *userData)
{
    if (status == kStatus_FLEXCAN_TxIdle)
    {
        txComplete = true;
    }
}

/* Safe PHY Configuration with Timeout and Error Reporting */
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

    txFrame.id = FLEXCAN_ID_STD(0x555);
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.length = 0U;
    txFrame.edl = 0U;
    txFrame.brs = 0U;

    txXfer.mbIdx = 0;
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
        /* UPDATED: Explicit Error Message */
        LOG_INFO("Transmission Error: PHY Config Timeout (No Bus ACK).\r\n");
        FLEXCAN_TransferFDAbortSend(EXAMPLE_CAN, &flexcanHandle, 0);
    }
    else
    {
        LOG_INFO("PHY Config ACK Received (Transmission Success).\r\n");
    }

    GPIO_PortClear(EXAMPLE_STB_RGPIO, 1u << EXAMPLE_STB_RGPIO_PIN);
}

static void Init_Peripherals(void){
    flexcan_config_t flexcanConfig;
    flexcan_rx_mb_config_t mbConfig;
    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 1};

    BOARD_InitHardware();
    LOG_INFO("--- FRDM-MCXN947 Universal CAN Sniffer ---\r\n");

    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);

    FLEXCAN_GetDefaultConfig(&flexcanConfig);

    /* ------------------------------------------------------ */
    /* CRITICAL FIXES FOR MONITORING                          */
    /* ------------------------------------------------------ */

    /* 1. Enable Individual Masks so we can set a "Promiscuous" mask */
    flexcanConfig.enableIndividMask = true;


    /* 3. Explicitly set Baud Rate (Match your Master!) */
    /* Default is often 1Mbps. Most tools use 500kbps. Adjust if needed. */
//    flexcanConfig.bitRate = 500000U;    // Nominal (Header) Speed
    flexcanConfig.bitRateFD = 2000000U; // Data Phase Speed (if FD)

    /* ------------------------------------------------------ */

#if defined(EXAMPLE_CAN_CLK_SOURCE)
    flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
#endif

    flexcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));

    /* Calculate Timing */
    if (FLEXCAN_FDCalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.bitRate, flexcanConfig.bitRateFD,
                                                EXAMPLE_CAN_CLK_FREQ, &timing_config))
    {
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }

    FLEXCAN_FDInit(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ, BYTES_IN_MB, true);

    /* Create Handle (Required for the TX function to work) */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* Do the Safe PHY Config (This was working, so keep it) */
    FLEXCAN_PHY_Config_Safe();

    /* Setup Receive Message Buffer (MB9) */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(0x000);

    FLEXCAN_SetFDRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);

    /* 4. Apply the "Allow Everything" Mask */
    /* MASK 0x000 means "Don't compare any bits" -> Accept ALL IDs */
    FLEXCAN_SetRxIndividualMask(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, FLEXCAN_RX_MB_STD_MASK(0x000, 0, 0));
}


int main(void)
{
    Init_Peripherals();

    LOG_INFO("Listening for ANY CAN message...\r\n");

    while (1)
    {

        /* Check MB Status Flags */
        if (FLEXCAN_GetMbStatusFlags(EXAMPLE_CAN, (1ULL << RX_MESSAGE_BUFFER_NUM)))
        {
            FLEXCAN_ClearMbStatusFlags(EXAMPLE_CAN, (1ULL << RX_MESSAGE_BUFFER_NUM));

            status_t status = FLEXCAN_ReadFDRxMb(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &rxFrame);

            if (status == kStatus_Success)
            {
            	// Collecting Data
            	uint32_t Destination_ID = rxFrame.id >> CAN_ID_STD_SHIFT;
            	uint32_t Data_Lenght = rxFrame.length;
            	uint8_t Data_Received = rxFrame.dataWord[0] >> 24;
            	uint8_t Source_ID = rxFrame.dataWord[0] >> 16;

            	LOG_INFO("Desitnation ID: 0x%02X | DLC: 0x%02X | Source ID: 0x%02X | Data: 0x%02X", Destination_ID, Data_Lenght, Source_ID, Data_Received);
                LOG_INFO("\r\n");

                /* UPDATED: Print ID and ALL Data Bytes */
//                LOG_INFO("RX ID: 0x%X | DLC: %d | Data: ",
//                       rxFrame.id >> CAN_ID_STD_SHIFT,
//                       rxFrame.length);


                /* Note: In FlexCAN FD, frame.length is the DLC code.
                   For standard CAN 0-8, it maps directly to bytes.
                   For FD > 8, you would typically use a helper to get bytes.
                   Here we print based on the raw DLC code assuming standard <=8 for simplicity
                   or use the dataWord array. */

				/*LOG_INFO("0x%02X ", rxFrame.dataWord[0]);*/
                if(Destination_ID == SLAVE_SERVO){
                	if(Source_ID == MASTER_DHT){
                		LOG_INFO("\tServo Motor received %d %% humidity from DHT22 sensor", Data_Received);
                	}
                	else if (Source_ID == MASTER_POTENTIOMETER){
                		LOG_INFO("\tServo Motor received %d %% from Potentiometer", Data_Received);
                	}
                }
                else if(Destination_ID == SLAVE_LED){
                	if(Source_ID == MASTER_DHT){
                		LOG_INFO("\tLED received %d C from DHT22 sensor", Data_Received);
                	}
                	else if (Source_ID == MASTER_POTENTIOMETER){
                		LOG_INFO("\tLED received %d %% from Potentiometer", Data_Received);
                	}
                }
                LOG_INFO("\r\n");


            }
            else
            {
                /* UPDATED: Print Error if Read Fails */
                LOG_INFO("Error receiving message. Status code: %d\r\n", status);
            }
        }
    }
}
