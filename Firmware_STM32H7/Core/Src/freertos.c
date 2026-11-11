/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "spi.h"
#include "gpio.h"
#include "fdcan.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// === AS5047P SPI DMA Variables ===
// Corrected Angle Register = 0x3FFF and set Bit 14 to 1 (for read) and set Bit 15 to 1 (for even parity bit)
uint16_t spi1_tx_buf = 0xFFFF; // 0xFFF = 0x3FFF + Read Bit + Even Parity
uint16_t spi1_rx_buf = 0;

volatile uint16_t encoder_raw_14bit = 0; // 0 to 15383
volatile float encoder_angle_deg = 0.0f; 
volatile uint8_t encoder_error_flag = 0; // 1 if parity or framing error detected
volatile uint8_t user_btn_pressed = 0; // button is active low on PB9
volatile uint32_t spi_dma_callback_cnt = 0; // increments at 1000Hz if DMA is healthy
volatile uint32_t vesc_debug_any_rx_cnt = 0;

// === VESC FDCAN Settings and Telemetry Variables ===
#define VESC_CAN_ID 108
#define MOTOR_POLE_PAIRS 14

typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT = 1,
  CAN_PACKET_SET_BRAKE = 2,
  CAN_PACKET_SET_RPM = 3,
  CAN_PACKET_STATUS = 9
} VESC_CAN_Packet_ID;

volatile float vesc_erpm = 0.0f; // ERPM reported by VESC
volatile float vesc_mech_rpm = 0.0f; // actual RPM
volatile float vesc_current_amps = 0.0f; // Iq reported by VESC
volatile float vesc_duty_cycle = 0.0f; // Duty cycle (-1.0 to 1.0)
volatile uint32_t vesc_rx_cnt = 0; // Increments every time VESC Status 1 arrives
volatile uint32_t vesc_tx_err_cnt = 0; // Stays 0 if CAN bus ACK / wiring is healthy

// helper function to check 16-bit even parity from AS5047P
static inline uint8_t check_even_parity(uint16_t val) {
  // perform XOR on all bits where 0 = even parity and 1 = odd parity
  val ^= val >> 8;
  val ^= val >> 4;
  val ^= val >> 2;
  val ^= val >> 1;
  return (val & 1); 
}
/* USER CODE END Variables */
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void VESC_CAN_Init(void);
void VESC_Set_Mech_RPM(float mech_rpm);
void VESC_Set_Current_Amps(float current_amps);
/* USER CODE END FunctionPrototypes */

void StartControlTask(void *argument);
void StartTelemetryTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the ControlTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  
  // Make sure CS (PC4) start HIGH
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
  
  // Start FDCAN1 peripheral & RX interrupts
  VESC_CAN_Init();
  osDelay(10);

  uint32_t tick = osKernelGetTickCount();
  uint32_t loop_div = 0;

  /* Infinite loop */
  for(;;)
  {
    tick += 1;
    osDelayUntil(tick);

    // Assert Chip Select (PC4 -> LOW)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    // Trigger non-blocking 16-bit SPI DMA transfer
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi1_tx_buf, (uint8_t*)&spi1_rx_buf, 1);

    // Send VESC CAN Command at 50Hz
    if (loop_div % 20 == 0) {
      if (user_btn_pressed) {
        VESC_Set_Mech_RPM(300.0f);
      } else {
        VESC_Set_Current_Amps(0.0f);
      }
    }
    loop_div++; 
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "\r\n=== RWIP Telmetry ===\r\n");
  uint32_t loop_cnt = 0;
  /* Infinite loop */
  for(;;)
  {
    // Read user button on PB9 (Active low)
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {
      user_btn_pressed = 1;
      // Turn on Green LED (PD7) while button is held down
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
    } else {
      user_btn_pressed = 0;
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    // Blink Red Heartbeat LED (PD5) every 250ms
    if (loop_cnt % 5 == 0) {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5);
    }

    // Print Telemetry over SEGGER RTT every 100ms
    if (loop_cnt % 2 == 0) {
      int angle_int = (int)encoder_angle_deg;
      int angle_frac = (int)((encoder_angle_deg - (float)angle_int) * 100.0f);
      int rpm_int = (int)vesc_mech_rpm;
      int curr_mA = (int)(vesc_current_amps * 1000.0f);
      SEGGER_RTT_printf(0, "BTN: %d | Angle: %3d.%02d deg | RPM:%5d | Iq: %5dmA | CAN_RX: %lu | CAN_Err: %lu | Any RX: %lu\r\n",
                        user_btn_pressed,
                        angle_int,
                        angle_frac,
                        rpm_int,
                        curr_mA,
                        vesc_rx_cnt,
                        vesc_tx_err_cnt,
                        vesc_debug_any_rx_cnt); 
    }
    
    loop_cnt++;
    osDelay(50);
  }
  /* USER CODE END StartTelemetryTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    // De-assert Chip Select (PC4 -> HIGH)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
    // check Bit 14 (error flag from AS5047P) and Bit 15 (Even Parity)
    if ((spi1_rx_buf & 0x4000) || (check_even_parity(spi1_rx_buf) != 0)) {
      encoder_error_flag = 1;
    } else {
      encoder_error_flag = 0;
      encoder_raw_14bit = spi1_rx_buf & 0x3FFF; // mask lower 14 bits
      encoder_angle_deg = ((float)encoder_raw_14bit * 360.0f) / 16384.0f;
    }
    spi_dma_callback_cnt++;
  }
}

// Initialize FDCAN1 Filter, Interrupts, and Start Peripheral
void VESC_CAN_Init(void) {
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // Accept all Extended IDs into RX FIFO 0
  sFilterConfig.FilterID2 = 0x00000000;

  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);
}

static void VESC_Send_Packet_Int32(uint8_t controller_id, VESC_CAN_Packet_ID packet_id, int32_t value) {
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[4];

  TxHeader.Identifier = ((uint32_t)packet_id << 8) | controller_id;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  // Pack int32 in Big-Endian order (MSB first, required by VESC)
  TxData[0] = (uint8_t)((value >> 24) & 0xFF);
  TxData[1] = (uint8_t)((value >> 16) & 0xFF);
  TxData[2] = (uint8_t)((value >> 8) & 0xFF);
  TxData[3] = (uint8_t)(value & 0xFF);

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0) {
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK) {
      vesc_tx_err_cnt++;
    }
  } else {
    vesc_tx_err_cnt++; // TX FIFO full (usually means no CAN ACK from VESC -> check wiring)
  }
}

// Command VESC Velocity in Mechanical RPM
void VESC_Set_Mech_RPM(float mech_rpm) {
  int32_t erpm = (int32_t)(mech_rpm * (float)MOTOR_POLE_PAIRS);
  VESC_Send_Packet_Int32(VESC_CAN_ID, CAN_PACKET_SET_RPM, erpm);
}

// Command VESC Current/Torque
void VESC_Set_Current_Amps(float current_amps) {
  int32_t current_mA = (int32_t)(current_amps * 1000.0f);
  VESC_Send_Packet_Int32(VESC_CAN_ID, CAN_PACKET_SET_CURRENT, current_mA);
}

// FDCAN_RX_Interrupt_Callback (Parses VESC CAN_PACKET_STATUS_1 automatically)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
      vesc_debug_any_rx_cnt++;
      if (RxHeader.IdType == FDCAN_EXTENDED_ID) {
        uint8_t sender_id = RxHeader.Identifier & 0xFF;
        uint8_t packet_id = (RxHeader.Identifier >> 8) & 0xFF;

        if (sender_id == VESC_CAN_ID && packet_id == CAN_PACKET_STATUS) {
          int32_t raw_erpm = (int32_t)(((uint32_t)RxData[0] << 24) |
                                       ((uint32_t)RxData[1] << 16) |
                                       ((uint32_t)RxData[2] << 8) |
                                       ((uint32_t)RxData[3]));
          int16_t raw_curr = (int16_t)(((uint16_t)RxData[4] << 8) | RxData[5]);
          int16_t raw_duty = (int16_t)(((uint16_t)RxData[6] << 8) | RxData[7]);

          vesc_erpm = (float)raw_erpm;
          vesc_mech_rpm = vesc_erpm / (float)MOTOR_POLE_PAIRS;
          vesc_current_amps = (float)raw_curr / 10.0f;
          vesc_duty_cycle = (float)raw_duty / 1000.0f;
          vesc_rx_cnt++;
        }
      }
    }
  } 
}

/* USER CODE END Application */

