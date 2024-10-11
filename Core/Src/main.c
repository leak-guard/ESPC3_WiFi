/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "ring_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define esp_uart huart1
#define ESP_TRANSMIT_BUFF_SIZE 64
#define ESP_PARSE_BUFF_SIZE 64

char echo_off[] = "ATE0";
char init_wifi[] = "AT+CWINIT=1";
char wifi_mode[] = "AT+CWMODE=1";
char wifi_no_autoconnect[] = "AT+CWAUTOCONN=0";
char wifi_connect[] = "AT+CWJAP=\"DM_TT\",\"W0B35d8y\"";
char ntp_pool[] = "AT+CIPSNTPCFG=1,2,\"pl.pool.ntp.org\"";
char query_time[] = "AT+CIPSNTPTIME?";
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
RingBuffer_t RxBuffer;
uint8_t UartRxTmp;
uint8_t NewLine;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int _write(int file, char *ptr, int len);

void ESP_Transmit_Log(char* s);
void ESP_Receive_Log(char* s);

void ESP_Transmit(char* s);

char* EspGetLine(void);
void ESP_WaitForOK(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&esp_uart, &UartRxTmp, 1);

  ESP_Transmit("AT");
  ESP_WaitForOK();

  ESP_Transmit(echo_off);
  ESP_WaitForOK();

  ESP_Transmit(init_wifi);
  ESP_WaitForOK();

  ESP_Transmit(wifi_mode);
  ESP_WaitForOK();

  ESP_Transmit(wifi_no_autoconnect);
  ESP_WaitForOK();

  ESP_Transmit(wifi_connect);
  ESP_WaitForOK();

  ESP_Transmit(ntp_pool);
  ESP_WaitForOK();

  ESP_Transmit(query_time);
  ESP_WaitForOK();

  ESP_Transmit(query_time);
  ESP_WaitForOK();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    __NOP();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
  int i;
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 50);
  for (i = 0; i < len; i++) {
    ITM_SendChar(*ptr++);
  }
  return len;
}

void ESP_Transmit_Log(char* s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)"T: ", 4, 1000);
  HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen((char*)s), 1000);
}

void ESP_Receive_Log(char* s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)"R: ", 4, 1000);
  HAL_UART_Transmit(&huart2, (uint8_t*)s, strlen((char*)s), 1000);
  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 4, 1000);
}

void ESP_Transmit(char* s)
{
  uint8_t Tmp[ESP_TRANSMIT_BUFF_SIZE] = {};
  strcat(Tmp, (char*)s);
  strcat(Tmp, "\r\n");
  ESP_Transmit_Log(Tmp);
  HAL_UART_Transmit(&esp_uart, (uint8_t*)Tmp, strlen(Tmp), 1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &esp_uart)
  {
    RB_Write(&RxBuffer, UartRxTmp);

    if (UartRxTmp == '\n')
    {
      NewLine++;
    }
    HAL_UART_Receive_IT(&esp_uart, &UartRxTmp, 1);
  }
}

char* EspGetLine(void)
{
  if (NewLine)
  {
    static char BuffTmp[ESP_PARSE_BUFF_SIZE];
    uint16_t i = 0;

    while (i < ESP_PARSE_BUFF_SIZE)
    {
      RB_Read(&RxBuffer, (uint8_t*)&BuffTmp[i]);

      if (BuffTmp[i] == '\r')
      {
        BuffTmp[i] = '\0';
      }

      if (BuffTmp[i] == '\n')
      {
        BuffTmp[i] = '\0';
        break;
      }
      i++;
    }
    NewLine--;

    return BuffTmp;
  }
  return 0;
}

void ESP_WaitForOK(void)
{
  char* BufferToParse;

  while (1)
  {
    BufferToParse = EspGetLine();

    if (BufferToParse)
    {
      ESP_Receive_Log(BufferToParse);

      if (0 == strncmp((char*)BufferToParse, "OK", ESP_PARSE_BUFF_SIZE))
      {
        break;
      }
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
