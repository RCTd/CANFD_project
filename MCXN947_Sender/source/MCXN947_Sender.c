/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_ctimer.h"
#include "fsl_sctimer.h" /* Added SCTimer Header */
#include "fsl_common.h"
#include "board.h"
#include "app.h"

/*******************************************************************************
 * Definitions & Settings
 ******************************************************************************/
#define FIFO_SIZE 16
/* RX_MESSAGE_BUFFER_NUM is defined in app.h */

/* --- LOGGING CONFIGURATION --- */
//#define LOGCHANGES

#ifdef LOGCHANGES
    #define LOG_STATE(...) LOG_INFO(__VA_ARGS__)
#else
    #define LOG_STATE(...)
#endif

typedef enum
{
    STATE_LISTEN,
    STATE_SEND,
    STATE_WAIT
} app_state_t;

typedef struct
{
    uint32_t id;
    uint8_t dataByte;
} can_msg_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* State Variables */
volatile app_state_t g_currentState = STATE_LISTEN;
volatile bool g_timerExpired = false;
volatile bool g_rxReceived = false;
volatile bool g_triggerSensorRead = false; /* Flag to trigger read from Main */


volatile dht_state_t g_dhtState = DHT_IDLE;
volatile uint32_t g_dhtBits[5];      /* Buffer for 40 bits (5 bytes) */
volatile uint8_t  g_dhtBitIdx = 0;   /* Current bit index (0-39) */
volatile uint32_t g_lastEdgeTime = 0;/* Timestamp of previous edge */

/* FIFO Variables */
volatile can_msg_t fifo_buffer[FIFO_SIZE];
volatile uint8_t fifo_head = 0;
volatile uint8_t fifo_tail = 0;

/* FlexCAN Handle and Variables */
flexcan_handle_t flexcanHandle;
flexcan_fd_frame_t txFrame;
flexcan_fd_frame_t rxFrame;
flexcan_mb_transfer_t txXfer;
flexcan_mb_transfer_t rxXfer;

volatile bool txComplete = true;
volatile bool txError = false;

static ctimer_match_config_t matchConfig0; /* For State Machine Timeout */

/* Forward declaration */
void Start_DHT_Read(void);

/*******************************************************************************
 * FIFO Helper Functions
 ******************************************************************************/
bool FIFO_Push(uint32_t id, uint8_t data)
{
    uint8_t next_head = (fifo_head + 1) % FIFO_SIZE;
    if (next_head == fifo_tail) return false;

    fifo_buffer[fifo_head].id = id;
    fifo_buffer[fifo_head].dataByte = data;
    fifo_head = next_head;
    return true;
}

bool FIFO_Pop(can_msg_t *msg)
{
    if (fifo_head == fifo_tail) return false;

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
    else if (status == kStatus_FLEXCAN_RxIdle)
    {
        g_rxReceived = true;
        rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM;
        rxXfer.framefd = &rxFrame;
        FLEXCAN_TransferFDReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
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

    if ((interruptFlags & (1U << BOARD_SW3_GPIO_PIN)) != 0)
    {
        GPIO_GpioClearInterruptFlags(BOARD_SW3_GPIO, 1U << BOARD_SW3_GPIO_PIN);
        FIFO_Push(TX_MSG_ID_RECEIVER2, 0x33);
    }

    if ((interruptFlags & (1U << BOARD_SW2_GPIO_PIN)) != 0)
    {
        GPIO_GpioClearInterruptFlags(BOARD_SW2_GPIO, 1U << BOARD_SW2_GPIO_PIN);
        Start_DHT_Read();
    }
    SDK_ISR_EXIT_BARRIER;
}

/* SCTimer Interrupt Handler for 2s Event */
//void SCT0_IRQHandler(void)
//{
//    /* Clear the Match Event Flag */
//    SCTIMER_ClearStatusFlags(SCT0, kSCTIMER_Event0Flag);
//
//    /* We set a flag here instead of reading the sensor.
//       Reading DHT22 takes ~20ms and blocks. Doing that in ISR is dangerous. */
//    g_triggerSensorRead = true;
//
//    SDK_ISR_EXIT_BARRIER;
//}

void SCT0_IRQHandler(void)
{
    uint32_t status = SCT0->EVFLAG;
    SCT0->EVFLAG = status; /* Clear flags */

    if (g_dhtState == DHT_START_18MS)
    {
        /* 18ms Low is done. Switch to Input/Listening Mode. */

        /* Stop forcing Low, let Pull-up take it High */
        GPIO_PinWrite(DHT_GPIO, DHT_PIN, 1U);

        /* Configure as Input */
        gpio_pin_config_t in_config = {kGPIO_DigitalInput, 0};
        GPIO_PinInit(DHT_GPIO, DHT_PIN, &in_config);

        /* FIXED: Use SetPinInterruptConfig instead of PortEnableInterrupts */
        GPIO_SetPinInterruptConfig(DHT_GPIO, DHT_PIN, kGPIO_InterruptEitherEdge);

        /* Enable IRQ in NVIC (Usually enabled in Init, but ensures it's on) */
        EnableIRQ(DHT_IRQn);

        /* Reset Timer to 0 and let it run free for timestamping */
        SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_U);
        SCT0->COUNT = 0;

        /* FIXED: Use EV[0] instead of EVENT[0] */
        SCT0->EV[0].STATE = 0;

        SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);

        g_dhtState = DHT_WAIT_RESPONSE;
    }

    SDK_ISR_EXIT_BARRIER;
}

void DHT_IRQ_HANDLER(void)
{
    /* FIX 1: Use GPIO_GpioGetInterruptFlags */
    if (GPIO_GpioGetInterruptFlags(DHT_GPIO) & (1U << DHT_PIN))
    {
        /* Clear Interrupt Flag */
        GPIO_GpioClearInterruptFlags(DHT_GPIO, 1U << DHT_PIN);

        /* Read current timestamp (in us) */
        uint32_t now = SCT0->COUNT;
        uint32_t width = now - g_lastEdgeTime;
        g_lastEdgeTime = now;

        /* Current Pin State */
        uint32_t pinState = GPIO_PinRead(DHT_GPIO, DHT_PIN);

        switch (g_dhtState)
        {
            case DHT_WAIT_RESPONSE:
                if (width > 60 && width < 100 && pinState == 0) {
                    g_dhtState = DHT_READING;
                    g_dhtBitIdx = 0;
                }
                break;

            case DHT_READING:
                if (pinState == 0)
                {
                    if (width > 40)
                    {
                        g_dhtBits[g_dhtBitIdx / 8] |= (1 << (7 - (g_dhtBitIdx % 8)));
                    }
                    g_dhtBitIdx++;

                    if (g_dhtBitIdx >= 40)
                    {
                        /* FIX 2: Pass 0 to disable the interrupt config */
                        GPIO_SetPinInterruptConfig(DHT_GPIO, DHT_PIN, (gpio_interrupt_config_t)0);

                        g_dhtState = DHT_IDLE;

                        uint8_t sum = g_dhtBits[0] + g_dhtBits[1] + g_dhtBits[2] + g_dhtBits[3];
                        if (g_dhtBits[4] == sum)
                        {
                            g_triggerSensorRead = true;
                        }
                        else
                        {
                            LOG_INFO("DHT Checksum Error\r\n");
                        }
                    }
                }
                break;

            default: break;
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

void Start_DHT_Read(void)
{
    if (g_dhtState != DHT_IDLE) return; /* Busy */

    LOG_INFO("Starting DHT22 Read (Async)...\r\n");

    /* 1. Reset State Variables */
    g_dhtState = DHT_START_18MS;
    g_dhtBitIdx = 0;
    memset((void*)g_dhtBits, 0, sizeof(g_dhtBits));

    /* 2. Pull Pin LOW for 18ms Start Signal */
    gpio_pin_config_t out_config = {kGPIO_DigitalOutput, 0};
    GPIO_PinInit(DHT_GPIO, DHT_PIN, &out_config);
    GPIO_PinWrite(DHT_GPIO, DHT_PIN, 0U);

    /* 3. Setup SCTimer for 18ms Timeout (One-Shot) */
    SCTIMER_StopTimer(SCT0, kSCTIMER_Counter_U);
    SCTIMER_Init(SCT0, &(sctimer_config_t){
        .enableCounterUnify = true,
        .clockMode = kSCTIMER_System_ClockMode,
        .prescale_l = (SystemCoreClock / 1000000U) - 1U /* FIXED: prescale_l (lowercase) */
    });

    /* Create Match Event for 18ms (18000us) */
    uint32_t eventId;
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_MatchEventOnly, 18000U, 0, kSCTIMER_Counter_U, &eventId);
    SCTIMER_EnableInterrupts(SCT0, 1 << eventId);
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);
}

/* Callback for CTIMER Match 0 - State Machine Timeout */
void ctimer_match0_callback(uint32_t flags)
{
    CTIMER_StopTimer(CTIMER);
    g_timerExpired = true;
}

ctimer_callback_t ctimer_callback_table[] = {
    ctimer_match0_callback, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};

/*******************************************************************************
 * Helper Functions
 ******************************************************************************/

static void Restart_Timer_ms(uint32_t ms)
{
    /* This manages Channel 0 for the State Machine */
    CTIMER_StopTimer(CTIMER);
    CTIMER_Reset(CTIMER);
    g_timerExpired = false;

    uint32_t matchVal = (CTIMER_CLK_FREQ / 1000U) * ms;
    matchConfig0.matchValue = matchVal;
    CTIMER_SetupMatch(CTIMER, CTIMER_MAT0_OUT, &matchConfig0);
    CTIMER_StartTimer(CTIMER);
}

static status_t SendCanMessageNonBlocking(uint32_t id, uint8_t dataByte)
{
    txFrame.id     = FLEXCAN_ID_STD(id);
    txFrame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    txFrame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
    txFrame.edl    = 1U;
    txFrame.brs    = 1U;
//    txFrame.length = 1U;
    txFrame.length = 2U;
//    txFrame.dataWord[0] = CAN_WORD_DATA_BYTE_0(dataByte);
    txFrame.dataWord[0] = (uint32_t)dataByte | (SENDER1 << 8);

    txXfer.mbIdx = TX_MESSAGE_BUFFER_NUM;
    txXfer.framefd = &txFrame;

    txComplete = false;

    LOG_INFO(">> Sending MSG (ID: 0x%X)...\r\n", id);

    status_t status = FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);

    if (status != kStatus_Success)
    {
        txComplete = true;
    }

    return status;
}

static void FLEXCAN_PHY_Config_Safe(void)
{
    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);

    gpio_pin_config_t stb_config = {kGPIO_DigitalOutput, 1};
    GPIO_PinInit(EXAMPLE_STB_RGPIO, EXAMPLE_STB_RGPIO_PIN, &stb_config);
    GPIO_PortSet(EXAMPLE_STB_RGPIO, 1u << EXAMPLE_STB_RGPIO_PIN);

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
    while (!txComplete && timeout > 0) timeout--;

    if (timeout == 0) FLEXCAN_TransferFDAbortSend(EXAMPLE_CAN, &flexcanHandle, TX_MESSAGE_BUFFER_NUM);

    GPIO_PortClear(EXAMPLE_STB_RGPIO, 1u << EXAMPLE_STB_RGPIO_PIN);
}

static void Init_SCTimer_2s(void)
{
    sctimer_config_t sctimerInfo;
    uint32_t eventCounter;
    uint32_t matchValue;

    SCTIMER_GetDefaultConfig(&sctimerInfo);

    /* Use Unified 32-bit counter to safely reach 2 seconds.
       16-bit counters on high speed clocks often overflow before 2s. */
    sctimerInfo.enableCounterUnify = true;
    sctimerInfo.clockMode = kSCTIMER_System_ClockMode;

    SCTIMER_Init(SCT0, &sctimerInfo);

    /* Calculate match value for 2000ms */
    matchValue = MSEC_TO_COUNT(2000U, SCTIMER_CLK_FREQ);

    /* Schedule the event. This returns the event ID into eventCounter */
    SCTIMER_CreateAndScheduleEvent(SCT0, kSCTIMER_MatchEventOnly, matchValue, 0, kSCTIMER_Counter_U, &eventCounter);

    /* Reset the counter when this event occurs (to make it periodic) */
    SCTIMER_SetupCounterLimitAction(SCT0, kSCTIMER_Counter_U, eventCounter);

    /* Enable Interrupts for this event */
    SCTIMER_EnableInterrupts(SCT0, 1 << eventCounter);

    /* Enable NVIC for SCTimer */
    EnableIRQ(SCT0_IRQn);

    /* Start the Timer */
    SCTIMER_StartTimer(SCT0, kSCTIMER_Counter_U);
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
    flexcan_rx_mb_config_t mbConfig;

    BOARD_InitHardware();

    LOG_INFO("--- CAN State Machine (SCTimer 2s Enabled) ---\r\n");

    /* GPIO */
    GPIO_SetPinInterruptConfig(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, kGPIO_InterruptFallingEdge);
    GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &sw_config);
    GPIO_SetPinInterruptConfig(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, kGPIO_InterruptFallingEdge);
    GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &sw_config);
    EnableIRQ(BOARD_SW3_IRQ);

    /* CTimer - Used for State Machine Timeouts */
    CTIMER_GetDefaultConfig(&ctimerConfig);
    CTIMER_Init(CTIMER, &ctimerConfig);

    matchConfig0.enableCounterReset = true;
    matchConfig0.enableCounterStop = false;
    matchConfig0.matchValue = CTIMER_CLK_FREQ * 1;
    matchConfig0.outControl = kCTIMER_Output_NoAction;
    matchConfig0.outPinInitState = false;
    matchConfig0.enableInterrupt = true;

    CTIMER_RegisterCallBack(CTIMER, &ctimer_callback_table[0], kCTIMER_MultipleCallback);
    EnableIRQ(CTIMER_IRQ_ID);

    /* SCTimer - Used for Periodic 2s Sensor Read */
    Init_SCTimer_2s();

    /* FlexCAN */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);
    flexcanConfig.disableSelfReception = true;
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
    FLEXCAN_PHY_Config_Safe();

    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);

    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(0);

    FLEXCAN_SetFDRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
    FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, FLEXCAN_RX_MB_STD_MASK(0, 0, 0));

    rxXfer.mbIdx = RX_MESSAGE_BUFFER_NUM;
    rxXfer.framefd = &rxFrame;
    FLEXCAN_TransferFDReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);

    txComplete = true;
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
    Init_Peripherals();
    can_msg_t nextMsg;

    g_currentState = STATE_LISTEN;
    LOG_STATE("[STATE] -> LISTEN (1000ms)\r\n");
    Restart_Timer_ms(LISTENTIMEMS);
    g_rxReceived = false;

    while (1)
    {
//        /* Check if SCTimer triggered a sensor read */
//        if (g_triggerSensorRead)
//        {
//            g_triggerSensorRead = false;
//
//            float humidity = 0.0f;
//            float temperature = 0.0f;
//
//            if (read_dht22(&humidity, &temperature)) {
//                LOG_INFO("Reading done \r\n");
//            } else {
//                LOG_INFO("Sensor Read Failed\r\n");
//            }
//            /* Push data as integers to CAN FIFO */
//            FIFO_Push(TX_MSG_ID_RECEIVER1, (uint8_t)humidity);
//            FIFO_Push(TX_MSG_ID_RECEIVER2, (uint8_t)temperature);
//        }
    	if (g_triggerSensorRead)
    	{
    	    g_triggerSensorRead = false;

    	    /* Convert Global Buffer to Values */
    	    float h_val = (float)((g_dhtBits[0] << 8) + g_dhtBits[1]) / 10.0f;
    	    float t_val = (float)((g_dhtBits[2] << 8) + g_dhtBits[3]) / 10.0f;

    	    LOG_INFO("DHT Async Read: H=%.1f T=%.1f\r\n", h_val, t_val);

    	    FIFO_Push(TX_MSG_ID_RECEIVER1, (uint8_t)h_val);
    	    FIFO_Push(TX_MSG_ID_RECEIVER2, (uint8_t)t_val);
    	}

        /***********************************************************************
         * STATE: LISTEN
         **********************************************************************/
        if (g_currentState == STATE_LISTEN)
        {
            if (g_rxReceived)
            {
                LOG_INFO("Event: CAN Msg Received (Listen)\r\n");
                g_rxReceived = false;
                g_currentState = STATE_WAIT;
                LOG_STATE("[STATE] -> WAIT (500ms)\r\n");
                Restart_Timer_ms(WAITTIMEMS);
            }
            else if (g_timerExpired)
            {
            	LOG_STATE("Event: Timer Expired (Listen)\r\n");
                g_currentState = STATE_SEND;
                LOG_STATE("[STATE] -> SEND (1000ms)\r\n");
                Restart_Timer_ms(SENDTIMEMS);
            }
        }

        /***********************************************************************
         * STATE: SEND
         **********************************************************************/
        else if (g_currentState == STATE_SEND)
        {
            bool fifoHasData;

            /* ABORT LOGIC */
            if (!txComplete)
            {
                LOG_INFO("Previous Tx Stuck. Aborting.\r\n");
                FLEXCAN_TransferFDAbortSend(EXAMPLE_CAN, &flexcanHandle, TX_MESSAGE_BUFFER_NUM);
                txComplete = true;
            }

            __disable_irq();
            fifoHasData = !FIFO_IsEmpty();
            __enable_irq();

            if (g_rxReceived)
            {
                LOG_INFO("Event: CAN Msg Received (Send)\r\n");
                g_rxReceived = false;
                g_currentState = STATE_WAIT;
                LOG_STATE("[STATE] -> WAIT (500ms)\r\n");
                Restart_Timer_ms(WAITTIMEMS);
            }
            else if (fifoHasData)
            {
                __disable_irq();
                bool popped = FIFO_Pop(&nextMsg);
                __enable_irq();

                if (popped)
                {
                    status_t txStatus = SendCanMessageNonBlocking(nextMsg.id, nextMsg.dataByte);

                    if (txStatus == kStatus_Success)
                    {
                        LOG_INFO("Tx Queued.\r\n");
                    }
                    else
                    {
                        LOG_INFO("Tx Failed (Stat: %d). Item Discarded.\r\n", txStatus);
                        txComplete = true;
                    }

                    g_currentState = STATE_LISTEN;
                    LOG_STATE("[STATE] -> LISTEN (1000ms)\r\n");
                    Restart_Timer_ms(LISTENTIMEMS);
                }
            }
            else if (g_timerExpired)
            {
            	LOG_STATE("Event: Timer Expired (Send)\r\n");
                g_currentState = STATE_LISTEN;
                LOG_STATE("[STATE] -> LISTEN (1000ms)\r\n");
                Restart_Timer_ms(LISTENTIMEMS);
            }
        }

        /***********************************************************************
         * STATE: WAIT
         **********************************************************************/
        else if (g_currentState == STATE_WAIT)
        {
            if (g_rxReceived)
            {
                LOG_INFO("Event: CAN Msg Received (Wait) -> Resetting Wait\r\n");
                g_rxReceived = false;
                Restart_Timer_ms(WAITTIMEMS);
            }
            else if (g_timerExpired)
            {
                LOG_INFO("Event: Wait Finished\r\n");
                g_currentState = STATE_SEND;
                LOG_STATE("[STATE] -> SEND (1000ms)\r\n");
                Restart_Timer_ms(SENDAFTERWAITTIMEMS);
            }
        }
    }
}
