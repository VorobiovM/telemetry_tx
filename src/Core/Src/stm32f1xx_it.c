/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_crc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_FRAME 0

#define UART_MSG_LEN (16)
#define DMA_UART_MSG_CNT (20)
#define DMA_BUFFER_SIZE (UART_MSG_LEN * DMA_UART_MSG_CNT)
#define CAN_MSG_LEN (8)
#define CRC_LEN (((CAN_MSG_LEN) * (sizeof(uint8_t))) / (sizeof(uint32_t)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
union CanData{
  uint8_t byte[CAN_MSG_LEN];
  uint32_t word[CRC_LEN];
};

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static union CanData can_data;
static CAN_RxHeaderTypeDef header;

static uint32_t can_crc = { 0 };

static uint8_t dma_buffer[DMA_BUFFER_SIZE] = { 0 };
static uint8_t dma_offset = 0;
static uint8_t *const p_buffer = (uint8_t *)&dma_buffer;
#if DEBUG_FRAME == 1
static uint32_t frame_count = 0;
static char frames[10] = { 0 };
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart1;
extern CRC_HandleTypeDef hcrc;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */
  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */
  HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, can_data.byte);

  if (HAL_CRC_GetState(&hcrc) == HAL_CRC_STATE_READY) {
    can_crc = HAL_CRC_Calculate(&hcrc, can_data.word, CRC_LEN);
  }

  // Fill DMA buffer
#if DEBUG_FRAME == 1
  // @formatter:off
  sprintf(frames, "%09u", ++frame_count);
  dma_buffer[UART_MSG_LEN * dma_offset +  0] = 'F';
  dma_buffer[UART_MSG_LEN * dma_offset +  1] = ':';
  dma_buffer[UART_MSG_LEN * dma_offset +  2] = frames[0];
  dma_buffer[UART_MSG_LEN * dma_offset +  3] = frames[1];
  dma_buffer[UART_MSG_LEN * dma_offset +  4] = frames[2];
  dma_buffer[UART_MSG_LEN * dma_offset +  5] = frames[3];
  dma_buffer[UART_MSG_LEN * dma_offset +  6] = frames[4];
  dma_buffer[UART_MSG_LEN * dma_offset +  7] = frames[5];
  dma_buffer[UART_MSG_LEN * dma_offset +  8] = frames[6];
  dma_buffer[UART_MSG_LEN * dma_offset +  9] = frames[7];
  dma_buffer[UART_MSG_LEN * dma_offset + 10] = frames[8];
  dma_buffer[UART_MSG_LEN * dma_offset + 11] = frames[9];
  dma_buffer[UART_MSG_LEN * dma_offset + 12] = 'O';
  dma_buffer[UART_MSG_LEN * dma_offset + 13] = ':';
  dma_buffer[UART_MSG_LEN * dma_offset + 14] = (dma_offset / 10) + 48;
  dma_buffer[UART_MSG_LEN * dma_offset + 15] = (dma_offset % 10) + 48;
    // @formatter:on
#else
  // @formatter:off
  dma_buffer[UART_MSG_LEN * dma_offset +  0] = 0xFE; // Start padding
  dma_buffer[UART_MSG_LEN * dma_offset +  1] = (header.StdId & 0x000000FF) >> 0;
  dma_buffer[UART_MSG_LEN * dma_offset +  2] = (header.StdId & 0x0000FF00) >> 8;
  dma_buffer[UART_MSG_LEN * dma_offset +  3] = can_data.byte[0];
  dma_buffer[UART_MSG_LEN * dma_offset +  4] = can_data.byte[1];
  dma_buffer[UART_MSG_LEN * dma_offset +  5] = can_data.byte[2];
  dma_buffer[UART_MSG_LEN * dma_offset +  6] = can_data.byte[3];
  dma_buffer[UART_MSG_LEN * dma_offset +  7] = can_data.byte[4];
  dma_buffer[UART_MSG_LEN * dma_offset +  8] = can_data.byte[5];
  dma_buffer[UART_MSG_LEN * dma_offset +  9] = can_data.byte[6];
  dma_buffer[UART_MSG_LEN * dma_offset + 10] = can_data.byte[7];
  dma_buffer[UART_MSG_LEN * dma_offset + 11] = (can_crc & 0x000000FF) >> 0;  // CRC-32 CAN data
  dma_buffer[UART_MSG_LEN * dma_offset + 12] = (can_crc & 0x0000FF00) >> 8;  // CRC-32 CAN data
  dma_buffer[UART_MSG_LEN * dma_offset + 13] = (can_crc & 0x00FF0000) >> 16; // CRC-32 CAN data
  dma_buffer[UART_MSG_LEN * dma_offset + 14] = (can_crc & 0xFF000000) >> 24; // CRC-32 CAN data
  dma_buffer[UART_MSG_LEN * dma_offset + 15] = 0x7F; // End padding
    // @formatter:on
#endif
  // Send USART via DMA
  if (++dma_offset == DMA_UART_MSG_CNT) {
    dma_offset = 0;
    HAL_UART_Transmit_DMA(&huart1, p_buffer, DMA_BUFFER_SIZE);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
  }
  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
