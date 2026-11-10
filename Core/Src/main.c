#include "main.h"
#include "stm32f0xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ---- Handles ---- */
UART_HandleTypeDef huart2;
CAN_HandleTypeDef  hcan;

/* ---- Prototypes ---- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);

/* ---- Tiny printf to UART ---- */
static void uprintf(const char *fmt, ...)
{
  char buf[128];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(buf)) n = sizeof(buf);
  HAL_UART_Transmit(&huart2, (uint8_t*)buf, (uint16_t)n, HAL_MAX_DELAY);
}

/* ======================= MAIN ======================= */
int main(void)
{
  HAL_Init();
  SystemClock_Config();    // 48 MHz
  MX_GPIO_Init();
  MX_USART2_UART_Init();   // VCP → PuTTY
  MX_CAN_Init();           // CAN on PA11/PA12

  /* Configure CAN filter: accept all IDs to FIFO0 */
  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow  = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow  = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &filter);

  /* Start CAN + enable RX interrupt */
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    uprintf("CAN start failed\r\n");
    while (1);
  }
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  uprintf("CAN ready (1Mbps @ 48MHz). Waiting for frames...\r\n");

  while (1) {
    /* your app can sleep or do other work here */
    HAL_Delay(50);
  }
}

/* ============ CAN RX interrupt callback ============ */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_)
{
  CAN_RxHeaderTypeDef rxh;
  uint8_t data[8];
  if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &rxh, data) == HAL_OK)
  {
    if (rxh.IDE == CAN_ID_STD) {
      uprintf("ID:0x%03lX ", (uint32_t)rxh.StdId);
    } else {
      uprintf("ID:0x%08lX ", (uint32_t)rxh.ExtId);
    }
    uprintf("DLC:%u DATA:", rxh.DLC);
    for (uint8_t i = 0; i < rxh.DLC; i++) uprintf(" %02X", data[i]);
    uprintf("\r\n");
  }
}

/* ================== Peripheral inits ================= */

/* 48 MHz system clock using HSI48 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Enable 48 MHz internal RC */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State     = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  /* Set SYSCLK = 48 MHz, HCLK = 48 MHz, PCLK1 = 48 MHz */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK |
                                     RCC_CLOCKTYPE_SYSCLK |
                                     RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    Error_Handler();
}


/* GPIO (CubeMX would usually generate this) */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /* No manual GPIO needed here; CAN/USART pins get configured by their inits */
}

/* USART2 (Virtual COM, PA2=TX, PA15=RX on Nucleo-32) */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate   = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits   = UART_STOPBITS_1;
  huart2.Init.Parity     = UART_PARITY_NONE;
  huart2.Init.Mode       = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) while (1);
}

/* CAN on PA11(CAN_RX) / PA12(CAN_TX) */
static void MX_CAN_Init(void)
{
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;    // 48MHz / (6 * (1+5+2)) = 1 Mbps
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff        = ENABLE;
  hcan.Init.AutoWakeUp        = ENABLE;
  hcan.Init.AutoRetransmission= ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) while (1);
}

/* The F0 maps CAN IRQ to CEC_CAN_IRQn */
void CEC_CAN_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

/* Simple trap for fatal errors */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {
    /* optional: blink an LED here so you can see it locked up */
  }
}

