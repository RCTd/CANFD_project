/*
 * Copyright 2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* --- CAN FD Definitions --- */
#define USE_CANFD                     (1U)
#define EXAMPLE_CAN                   CAN0
#define EXAMPLE_CAN_CLK_SOURCE        (kFLEXCAN_ClkSrc0)
#define EXAMPLE_CAN_CLK_FREQ          CLOCK_GetFlexcanClkFreq(0U)
#define USE_IMPROVED_TIMING_CONFIG    (1U)
#define TX_MESSAGE_BUFFER_NUM         (8U)

/* Receiver Configuration */
#define RX_MESSAGE_BUFFER_NUM         (9U)  /* Use MB9 for reception */
#define RX_MSG_ID                     0x321UL /* Listening for this ID */
#define BYTES_IN_MB                   kFLEXCAN_8BperMB
#define DLC                           (1U)

/* --- LED Definitions (Red LED) --- */
#define BOARD_LED_GPIO                BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN            BOARD_LED_RED_GPIO_PIN
/* IDs for the messages */
#define TX_MSG_ID_BUTTON3             0x123UL
#define TX_MSG_ID_BUTTON2             0x124UL
#define TX_MSG_ID_TIMER               0x321UL

/* --- CTimer Definitions --- */
#define CTIMER                        CTIMER4
#define CTIMER_CLK_FREQ               CLOCK_GetCTimerClkFreq(4U)
#define CTIMER_MAT0_OUT               kCTIMER_Match_0
#define CTIMER_IRQ_HANDLER            CTIMER4_IRQHandler
#define CTIMER_IRQ_ID                 CTIMER4_IRQn

/* --- TJA1152 Transceiver --- */
#define EXAMPLE_STB_RGPIO             GPIO3
#define EXAMPLE_STB_RGPIO_PIN         13U

//Fir Roșu - Pin 8 (P0_24)
//Fir Galben -Pin 10 (P0_26)
//Fir Verde - Pin 12 (P0_25)
#define RED_PORT GPIO0
#define RED_PIN  24U // J2-8

#define YEL_PORT GPIO0
#define YEL_PIN  26U // J2-10

#define GRN_PORT GPIO0
#define GRN_PIN  25U // J2-12

typedef enum { STATE_OFF, STATE_RED, STATE_YEL, STATE_GRN } traffic_state_t;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);

#ifndef LOG_INFO
#define LOG_INFO (void)PRINTF
#endif

#endif /* _APP_H_ */
