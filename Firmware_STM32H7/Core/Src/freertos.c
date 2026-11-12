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
#include "math.h"
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
volatile int32_t encoder_zero_offset_raw = 0; // 0 deg = down, 180 deg = upright
volatile float pendulum_angle_deg = 0.0f;
volatile float upright_error_deg = 0.0f;
volatile float pendulum_vel_deg_s = 0.0f; // filtered angular velocity
volatile uint8_t encoder_error_flag = 0; // 1 if parity or framing error detected
volatile uint32_t spi_dma_callback_cnt = 0; // increments at 1000Hz if DMA is healthy

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

// === Finite State Machine ===
typedef enum {
  UNCALIBRATED,
  READY,
  RUNNING
} RWIP_FSM;

volatile RWIP_FSM RWIP_state = UNCALIBRATED; 

// === Calibration Counter 
volatile uint16_t calib_sample_cnt = 0;
volatile uint32_t calib_accumulator = 0;
volatile uint8_t is_calibrating = 0;

// === User Button Edge Detection ===
volatile uint8_t user_btn_pressed = 0; // button is active low on PB9
uint8_t user_btn_prev = 0;

// === PID & Wheel Damping Gains
volatile float Kp = 0.2f; // to be tuned
volatile float Ki = 0.0f; // to be tuned
volatile float Kd = 0.03f; // to be tuned
volatile float Kw = 0.0f; // to be tuned, Wheel velocity damping (Amps per Mech RPM)

volatile float motor_dir = -1.0f; 
volatile float pid_integral = 0.0f;
volatile float commanded_current_amps = 0.0f;

#define MAX_CURRENT_AMPS 2.0f
#define SAFETY_CUTOFF_DEG 25.0f

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
  const float dt_ctrl = 0.005f; // 200Hz control output loop (5ms)

  /* Infinite loop */
  for(;;)
  {
    tick += 1;
    osDelayUntil(tick);

    // Assert Chip Select (PC4 -> LOW)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    // Trigger 16-bit SPI DMA transfer
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&spi1_tx_buf, (uint8_t*)&spi1_rx_buf, 1);

    // Run FSM Control and CAN Output at 200Hz
    if (loop_div % 5 == 0) {
      if (RWIP_state == RUNNING) {
        // check if pendulum fall past safety cutoff of +/- 25 deg
        if (fabsf(upright_error_deg) > SAFETY_CUTOFF_DEG) {
          commanded_current_amps = 0.0f;
          pid_integral = 0.0f;
          RWIP_state = READY;
          VESC_Set_Current_Amps(0.0f);
        } else {
          // accumulate integral error with anti-windup
          pid_integral += upright_error_deg * dt_ctrl;
          if (pid_integral > 10.0f) {
            pid_integral = 10.0f;
          }
          if (pid_integral < -10.0f) {
            pid_integral = -10.0f;
          }

          // PID control + Wheel velocity damping control law
          float u_amps = (Kp * upright_error_deg) + 
                         (Ki * pid_integral) + 
                         (Kd * pendulum_vel_deg_s) +
                         (Kw * vesc_mech_rpm);
          
          // apply motor direction sign and clamp current
          u_amps *= motor_dir;
          if (u_amps > MAX_CURRENT_AMPS) {
            u_amps = MAX_CURRENT_AMPS;
          }
          if (u_amps < -MAX_CURRENT_AMPS) {
            u_amps = -MAX_CURRENT_AMPS;
          }

          commanded_current_amps = u_amps;
          VESC_Set_Current_Amps(u_amps);
        }
      } else {
        // RWIP_state in UNCALIBRATED or READY state
        commanded_current_amps = 0.0f;
        pid_integral = 0.0f;
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
    } else {
      user_btn_pressed = 0;
    }

    // FSM button handling
    if (user_btn_pressed && !user_btn_prev) {
      // button was just pressed
      switch (RWIP_state) {
        case UNCALIBRATED:
          if (!is_calibrating) {
            SEGGER_RTT_WriteString(0, "Calibrating....\r\n");
            calib_accumulator = 0;
            calib_sample_cnt = 0;
            is_calibrating = 1; 
          }
          break;
        
        case READY:
          if (fabsf(upright_error_deg) < 15.0f) {
            pid_integral = 0.0f;
            RWIP_state = RUNNING;
            SEGGER_RTT_WriteString(0, "Running!!!\r\n");
          } else {
            SEGGER_RTT_WriteString(0, "Please move pendulum +/- 15 deg from upright position!\r\n");
          }
          break;

        case RUNNING:
          RWIP_state = READY;
          SEGGER_RTT_WriteString(0, "Motor Disarming!\r\n");
          break;
      }
    }

    user_btn_prev = user_btn_pressed;

    // FSM Green LED handling (PD7)
    switch (RWIP_state) {
      case UNCALIBRATED:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET); // turn off LED
        break;
      case READY:
        if (loop_cnt % 5 == 0) { // Blink at 2Hz (toggle every 250ms)
          HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_7);
        }
        break;
      case RUNNING:
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET); // turn on LED
        break;
    }

    // Blink Red Heartbeat LED (PD5) at 2Hz (toggle every 250ms)
    if (loop_cnt % 5 == 0) {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5);
    }

    // Print Telemetry over SEGGER RTT at 10Hz (100ms)
    if (loop_cnt % 2 == 0) {
      int angle_int = (int)pendulum_angle_deg;
      int err_int = (int)(upright_error_deg * 100.0f);
      int cmd_mA = (int)(commanded_current_amps * 1000.0f);
      int vesc_rpm = (int)vesc_mech_rpm;
      int vesc_curr_mA = (int)(vesc_current_amps * 1000.0f);
      SEGGER_RTT_printf(0, "State:%d | Angle:%3d deg | TopErr:%4d cDeg | CmdI:%5d mA | VescRPM:%5d | VescCur:%5d mA\r\n",
                        (int)RWIP_state, 
                        angle_int,
                        err_int,
                        cmd_mA,
                        vesc_rpm,
                        vesc_curr_mA); 
    }
    
    loop_cnt++;
    osDelay(50); // 20Hz loop (50ms)
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
      
      // handle 200 sample zero calibration
      if (is_calibrating) {
        calib_accumulator += encoder_raw_14bit;
        calib_sample_cnt++;
        if (calib_sample_cnt >= 200) {
          encoder_zero_offset_raw = (int32_t)(calib_accumulator) / (int32_t)(calib_sample_cnt);
          is_calibrating = 0;
          RWIP_state = READY;
        }
      }

      int32_t cal_raw = ((int32_t)encoder_raw_14bit - encoder_zero_offset_raw);
      cal_raw = cal_raw & 0x3FFF; // wrap cal_raw (0 to 16383) using & 0x3FFF
      pendulum_angle_deg = ((float)cal_raw * 360.0f) / 16384.0f;

      // copmute signed upright error
      int32_t err_counts = cal_raw - 8192; // err_counts will be (-8192 to 8192) 
      float new_upright_err_deg = ((float)err_counts * 360.0f) / 16384.0f;

      // compute derivative / velocity (dt = 0.001 at 1kHz) with low pass filter
      static int32_t prev_cal_raw = 0;
      int32_t delta_counts = cal_raw - prev_cal_raw;
      if (delta_counts > 8192) { // wrap delta_counts (-8192 to 8192)
        delta_counts -= 16384;
      } else if (delta_counts < -8192) {
        delta_counts += 16384;
      }
      prev_cal_raw = cal_raw;

      float raw_vel_deg_s = ((float)delta_counts * 360.0f / 16384.0f) * 1000.0f; // *1000 for deg/ms to deg/s
      float alpha = 0.15; // low pass filter
      pendulum_vel_deg_s = ((1.0f - alpha) * pendulum_vel_deg_s) + (alpha * raw_vel_deg_s);

      upright_error_deg = new_upright_err_deg;
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

