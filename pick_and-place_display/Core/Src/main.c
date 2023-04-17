/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define Relay_1_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define Relay_1_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)

#define Relay_2_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define Relay_2_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)

#define Relay_3_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define Relay_3_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)

#define Relay_4_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)
#define Relay_4_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)

#define SUCTION_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)
#define SUCTION_OFF HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int Rx_data[10];
int count[5];
int real_count = 0;
int start = 0;
char msg[100];
char data[10][10];
char splitStrings[10][10]; //can store 10 words of 10 characters
int i, j, cnt;
int init_size = 0;
char delim[] = ",";

char *str1[5];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void goright() {
	Relay_1_ON;
	Relay_2_OFF;
}

void goleft() {
	Relay_1_OFF;
	Relay_2_ON;
}

void go_up() {
	Relay_3_ON;
	Relay_4_OFF;
}

void go_down() {
	Relay_3_OFF;
	Relay_4_ON;
}

void initialization() {
	goright();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	go_up();
	HAL_Delay(5000);
}

void turnoff_init() {
	Relay_1_OFF;
	Relay_2_OFF;
	Relay_3_OFF;
	Relay_4_OFF;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

void turnoff() {
	Relay_1_OFF;
	Relay_2_OFF;
	Relay_3_OFF;
	Relay_4_OFF;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
//	count = 0;
//	dlay = 0;
	initialization();
	turnoff_init();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
//	HAL_Delay(10000)
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (HAL_UART_Receive(&huart1, Rx_data, 10, 1500) == HAL_OK) {
			HAL_UART_Transmit(&huart2, Rx_data, 10, 1500);
			HAL_Delay(1000);

			init_size = strlen(Rx_data);
			char *ptr = strtok(Rx_data, delim);

			while (ptr != NULL) {
				for (int i = 0; i <= 4; i++) {

					// printf("'%s'\n", ptr);
					str1[i] = ptr;
//					printf("'%s'\n", str1[i]);

//					sprintf(msg, "%s,%d\r\n", str1[i], i);
//					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
//							100);
					count[i] = atoi(str1[i]);
					sprintf(msg, "count=%d\r\n", count[i]);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							100);
					ptr = strtok(NULL, delim);

				}
			}
			for (int i = 0; i < 5; i++) {
				count[i] = atoi(str1[i]);
				sprintf(msg, "cou=%d\r\n", count[i]);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
			}
		}

//		HAL_UART_Transmit(&huart2, Rx_data, sizeof(Rx_data), 100);

		if (count[0] != 0 && count[1] != 0) {
			count[1] = count[1] * 1000; // up
			count[2] = count[2] * 1000; //down
			count[3] = count[3] * 1000; //right
			sprintf(msg,
					"inside count loop... count=%d,delayup= %d,delaydown= %d,delayright= %d\r\n",
					count[0], count[1], count[2], count[3]);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);

			//data received from arduino
			start = 1;

		}
		while (start == 1) {
			if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_SET)) {
				sprintf(msg,
						"inside start loop... count=%d,delayup= %d,delaydown= %d,delayright= %d\r\n",
						count[0], count[1], count[2], count[3]);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), 100);
				while (count[0] > real_count) {
					go_down(); // coming down R3 off, R4 On
					HAL_Delay(count[2]);
					turnoff();
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); //suction on
					go_up(); //Go Up, R3 On, R4 off
					HAL_Delay(count[1]);
					turnoff();
					goleft();    ///// go left R1 Off, R2 On
					HAL_Delay(count[3]);
					turnoff();
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //suction off
					HAL_Delay(200);
					goright();      // Go right R1 On, R2 Off
					HAL_Delay(count[3]);
					turnoff();
					real_count++;
					sprintf(msg,
							"inside real loop... count=%d,delayup= %d,delaydown= %d,delayright= %d,real_count=%d\r\n",
							count[0], count[1], count[2], count[3], real_count);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							100);
					sprintf(msg,"real_count=%d\r\n",real_count);
					HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg),
							100);
				}
				if (count[0] <= real_count) {
					start = 0;
					real_count = 0;
					count[0] = 0;
					count[1] = 0;
					HAL_UART_Transmit(&huart1, "a", 1, 100);
					sprintf(msg,
							"inside final loop...  count=%d,delayup= %d,delaydown= %d,delayright= %d,real_count=%d\r\n",
							count[0], count[1], count[2], count[3], real_count);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							100);

				}
			}
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);

	/*Configure GPIO pins : PA6 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB3 PB5 PB6 PB7
	 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
			| GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PE0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
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

