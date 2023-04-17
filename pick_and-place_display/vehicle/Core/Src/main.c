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
#include "string.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DELAY_SIDE HAL_Delay(32000)
#define DELAY_PLATFORM HAL_Delay(53000)
#define DELAY_SHUTTER HAL_Delay(20000)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int shutter = 0;
int side = 0;
int platform = 0;
int shut_flag = 0;
int side_flag = 0;
int plat_flag = 0;
int condition_1 = 0;
int condition_0 = 0;
int condition_2 = 0;
int forward = 1;
int backward = 0;
int condition_3 = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void shutter_open() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
//	GPIOD->ODR = 0b0000000001000000;

}

void shutter_close() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
}

void shutter_off() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
}

void side_open() {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

}

void side_close() {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

}

void side_off() {

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

}

void platform_rise() {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

}

void platform_down() {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);

}

void platform_off() {

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);

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
	/* USER CODE BEGIN 2 */
	HAL_UART_Transmit(&huart1, "y", 1, 100);
	/* USER CODE END 2 */
	side_off();
	platform_off();

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		while (forward == 1) {
//			HAL_UART_Transmit(&huart1, "forward", 10, 100);
			/*if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
			 HAL_Delay(50);
			 while ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
			 side_flag = !side_flag;
			 HAL_UART_Transmit(&huart1, "a", 1, 100);
			 //				SIDE_OPEN;
			 if (side_flag == 1) {
			 condition_3 = 0;
			 } else if (side_flag == 0) {
			 condition_3 = 1;
			 }
			 while ((side_flag == 1) && (condition_3 == 0)) {
			 HAL_UART_Transmit(&huart1, "c", 1, 100);
			 //						HAL_UART_Transmit(&huart1, "side", 10, 100);
			 side_open();
			 HAL_Delay(70000);

			 side_off();
			 side = 1;
			 HAL_UART_Transmit(&huart1, "d", 1, 100);
			 shutter = 0;
			 condition_3 = 1;
			 }
			 while ((side_flag == 0) && (condition_3 == 1)) {
			 HAL_UART_Transmit(&huart1, "c", 1, 100);
			 side_close();
			 HAL_Delay(70000);

			 side_off();
			 //			side = 1;
			 //			platform = 0;
			 HAL_UART_Transmit(&huart1, "b", 10, 100);
			 condition_3 = 0;
			 }

			 }
			 }*/

			while (condition_0 == 0) {

				if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
					HAL_Delay(50);
					while ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
						side_flag = !side_flag;
						HAL_UART_Transmit(&huart1, "a", 1, 100);
						//				SIDE_OPEN;
						if (side_flag == 1) {

							condition_3 = 0;
						} else if (side_flag == 0) {
							condition_3 = 1;
						}

						while ((side_flag == 1) && (condition_3 == 0)) {
							HAL_UART_Transmit(&huart1, "b", 1, 100);
							//						HAL_UART_Transmit(&huart1, "side", 10, 100);
							side_open();
//							HAL_Delay(70000);
							DELAY_SIDE;

							side_off();
							side = 1;
							HAL_UART_Transmit(&huart1, "c", 1, 100);
							//shutter = 0;
							condition_3 = 1;
						}
						while ((side_flag == 0) && (condition_3 == 1)) {
							HAL_UART_Transmit(&huart1, "b", 1, 100);
							side_close();
//							HAL_Delay(70000);
							DELAY_SIDE;

							side_off();
							//			side = 1;
							//			platform = 0;
							HAL_UART_Transmit(&huart1, "d", 10, 100);
							condition_3 = 0;
						}
					}
				}

				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
					HAL_Delay(50);
					while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
						shut_flag = !shut_flag;
						HAL_UART_Transmit(&huart1, "e", 1, 100);
						HAL_Delay(1000);
						//HAL_UART_Transmit(&huart1, "f	", 1, 100);
//						condition_0 = 1;

						while ((shut_flag == 1) && (shutter == 0)) {
							HAL_UART_Transmit(&huart1, "f", 1, 100);
							//							HAL_Delay(1000);
//					HAL_UART_Transmit(&huart1, "shutter1", 10, 100);
							shutter_open();

//							HAL_Delay(40000);
							DELAY_SHUTTER;

							shutter_off();
							HAL_UART_Transmit(&huart1, "g", 1, 100);
							shutter = 1;
							condition_1 = 1;
							condition_0 = 1;
						}
					}
				}

			}

			while ((condition_1 == 1) && (shutter == 1)) {
//				HAL_UART_Transmit(&huart1, "inside", 10, 100);
				if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
					HAL_Delay(50);
					while ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
						side_flag = !side_flag;
						HAL_UART_Transmit(&huart1, "a", 1, 100);
						//				SIDE_OPEN;
						if (side_flag == 1) {
							condition_3 = 0;
						} else if (side_flag == 0) {
							condition_3 = 1;
						}

						while ((side_flag == 1) && (condition_3 == 0)) {
							HAL_UART_Transmit(&huart1, "b", 1, 100);
							//						HAL_UART_Transmit(&huart1, "side", 10, 100);
							side_open();
//							HAL_Delay(70000);
							DELAY_SIDE;

							side_off();
							side = 1;
							HAL_UART_Transmit(&huart1, "c", 1, 100);
							//shutter = 0;
							condition_3 = 1;
						}
						while ((side_flag == 0) && (condition_3 == 1)) {
							HAL_UART_Transmit(&huart1, "b", 1, 100);
							side_close();
//							HAL_Delay(70000);
							DELAY_SIDE;

							side_off();
							//			side = 1;
							//			platform = 0;
							HAL_UART_Transmit(&huart1, "d", 10, 100);
							condition_3 = 0;
						}
					}
				}

				//check PA2 for side motor
				if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET)
						&& (condition_1 == 1)) {
					HAL_Delay(50);
					while ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET)
							&& (condition_1 == 1)) {
						plat_flag = !plat_flag;
						HAL_UART_Transmit(&huart1, "i", 1, 100);
						//				platform

						while ((plat_flag == 1) && (shutter == 1)) {
							HAL_UART_Transmit(&huart1, "j", 1, 100);
//				HAL_UART_Transmit(&huart1, "plat1", 10, 100);
							platform_rise();

//							HAL_Delay(70000);
							DELAY_PLATFORM;

							platform_off();
							HAL_UART_Transmit(&huart1, "k", 1, 100);
							side = 0;
//				condition_3 = 1;
							//side_flag = 0;
							shut_flag = 0;
							plat_flag = 0;
							shutter = 0;
							forward = 0;
							backward = 1;
							//HAL_UART_Transmit(&huart1, "m", 1, 100);
							condition_1 = 0;
						}
					}
				}

			}

		}
		while (backward == 1) {
//			HAL_UART_Transmit(&huart1, "backward", 10, 100);
			/*	if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
			 HAL_Delay(50);
			 while ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
			 side_flag = !side_flag;
			 HAL_UART_Transmit(&huart1, "a", 1, 100);
			 //				SIDE_OPEN;
			 if (side_flag == 1) {
			 condition_3 = 0;
			 } else if (side_flag == 0) {
			 condition_3 = 1;
			 }
			 while ((side_flag == 1) && (condition_3 == 0)) {
			 HAL_UART_Transmit(&huart1, "c", 1, 100);
			 //						HAL_UART_Transmit(&huart1, "side", 10, 100);
			 side_open();
			 HAL_Delay(70000);

			 side_off();
			 side = 1;
			 HAL_UART_Transmit(&huart1, "d", 1, 100);
			 shutter = 0;
			 condition_3 = 1;
			 }
			 while ((side_flag == 0) && (condition_3 == 1)) {
			 HAL_UART_Transmit(&huart1, "c", 1, 100);
			 side_close();
			 HAL_Delay(70000);

			 side_off();
			 //			side = 1;
			 //			platform = 0;
			 HAL_UART_Transmit(&huart1, "b", 10, 100);
			 condition_3 = 0;
			 }

			 }
			 }*/
			while (condition_0 == 1) {

				if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
					HAL_Delay(50);
					while ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
						side_flag = !side_flag;
						HAL_UART_Transmit(&huart1, "a", 1, 100);
						//				SIDE_OPEN;
						if (side_flag == 1) {
							condition_3 = 0;
						} else if (side_flag == 0) {
							condition_3 = 1;
						}

						while ((side_flag == 1) && (condition_3 == 0)) {
							HAL_UART_Transmit(&huart1, "b", 1, 100);
							//						HAL_UART_Transmit(&huart1, "side", 10, 100);
							side_open();
//							HAL_Delay(70000);
							DELAY_SIDE;
							side_off();
							side = 1;
							HAL_UART_Transmit(&huart1, "c", 1, 100);
							//shutter = 0;
							condition_3 = 1;
						}
						while ((side_flag == 0) && (condition_3 == 1)) {
							HAL_UART_Transmit(&huart1, "b", 1, 100);
							side_close();
//					HAL_Delay(70000);
							DELAY_SIDE;
							side_off();
							//			side = 1;
							//			platform = 0;
							HAL_UART_Transmit(&huart1, "d", 10, 100);
							condition_3 = 0;
						}
					}
				}

				if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) {
					HAL_Delay(50);
					while (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == GPIO_PIN_SET) {
						HAL_UART_Transmit(&huart1, "i", 1, 100);
						HAL_Delay(1000);
						//HAL_UART_Transmit(&huart1, "k", 1, 100);
//						HAL_UART_Transmit(&huart1, "platd", 10, 100);
						plat_flag = !plat_flag;
//						condition_0 = 0;

						while ((plat_flag == 1) && (platform == 0)) {
							HAL_UART_Transmit(&huart1, "j", 1, 100);
//					HAL_UART_Transmit(&huart1, "plat2", 10, 100);
							platform_down();
//							HAL_Delay(70000);
							DELAY_PLATFORM;
							platform_off();
							condition_1 = 1;
							HAL_UART_Transmit(&huart1, "l", 1, 100);
							platform = 1;
							condition_0 = 0;
						}
					}
				}

			}

			while ((condition_1 == 1) && (platform == 1)) {
				//check PA2 for side motor
				if ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
					HAL_Delay(50);
					while ((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)) {
						side_flag = !side_flag;
						HAL_UART_Transmit(&huart1, "a", 1, 100);
						//				SIDE_OPEN;
						if (side_flag == 1) {
							condition_3 = 0;
						} else if (side_flag == 0) {
							condition_3 = 1;
						}

						while ((side_flag == 1) && (condition_3 == 0)) {
							HAL_UART_Transmit(&huart1, "b", 1, 100);
							//						HAL_UART_Transmit(&huart1, "side", 10, 100);
							side_open();
//					HAL_Delay(70000);
							DELAY_SIDE;

							side_off();
							side = 1;
							HAL_UART_Transmit(&huart1, "c", 1, 100);
							//shutter = 0;
							condition_3 = 1;
						}
						while ((side_flag == 0) && (condition_3 == 1)) {
							HAL_UART_Transmit(&huart1, "b", 1, 100);
							side_close();
//					HAL_Delay(70000);
							DELAY_SIDE;

							side_off();
							//			side = 1;
							//			platform = 0;
							HAL_UART_Transmit(&huart1, "d", 10, 100);
							condition_3 = 0;
						}
					}
				}

				if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
						&& (condition_1 == 1)) {
					HAL_Delay(50);
					while ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
							&& (condition_1 == 1)) {
						shut_flag = !shut_flag;
						HAL_UART_Transmit(&huart1, "e", 1, 100);
//						HAL_UART_Transmit(&huart1, "sided", 10, 100);
						//				SIDE_OPEN;
						//condition_1 = 0;

						while ((shut_flag == 1) && (platform == 1)) {
							HAL_UART_Transmit(&huart1, "f", 1, 100);
//				HAL_UART_Transmit(&huart1, "side2", 10, 100);
							shutter_close();
//							HAL_Delay(40000);
							DELAY_SHUTTER;

							shutter_off();
							HAL_UART_Transmit(&huart1, "h", 1, 100);
							side = 0;
						//	side_flag = 0;
							shut_flag = 0;
							plat_flag = 0;
							platform = 0;

							forward = 1;
							backward = 0;
							condition_3 = 0;
							condition_0 = 0;
							//HAL_UART_Transmit(&huart1, "m", 1, 100);
							condition_1 = 0;
						}
					}
				}

			}

			//check PA2 for side motor

			/* USER CODE BEGIN 3 */
		}
		/* USER CODE BEGIN 3 */
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC0 PC1 PC2 PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PE13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : PD13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : PD6 PD7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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

