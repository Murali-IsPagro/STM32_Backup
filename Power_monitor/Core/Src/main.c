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
#include "i2c-lcd.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

#define Relay_on   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define Relay_off  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)

#define fault_on   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET)
#define fault_off  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET)
#define ADC_BUFLEN 4
#define MAXCURRENT  19
#define MAXVOLTAGE  260
#define MINVOLTAGE  0
#define INITCURRENT 5
//Voltage conversion factor
#define VOLTAGEFACTOR  0.05//0.23
float sensitivity = 0.1;
int rawvoltage = 0, rawcurrent = 0;
int rawlowVal = 0, rawhighVal = 0;
float current = 0, lowVal = 0, highVal = 0, tempcur = 0;
int voltage = 0;
int raw = 0;

int red = 0, sysState = 0, faultMon = 0, initOK = 0;
int faultCount = 0;
// Track time in milliseconds since last reading

char str1[20];

/* USER CODE END PTD */
uint16_t ADC_VAL[4];

ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
struct string {
	char str[20];
} s1, s2, s3, s4;

/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

void ADC_Select_CH0(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_CH1(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_Select_CH2(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_Select_CH4(void) {
	ADC_ChannelConfTypeDef sConfig = { 0 };
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void adc() {
	ADC_Select_CH0();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH2();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	ADC_Select_CH4();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 10);
	ADC_VAL[3] = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
}
void getCurrent(int readValue) {
	raw = (float) readValue * 3.3 * 2 / 4095;
	current = (raw - 2.5) / sensitivity;
}
float readvoltage() {
	adc();
	float tempVol = 0;
	for (int i = 0; i < 50; i++) {
		rawvoltage = ADC_VAL[3];
		tempVol += (rawvoltage * VOLTAGEFACTOR); //0.605 is found by trail and error. This factor is used to get the measure voltage.
	}
	tempVol = tempVol / 50;
	return tempVol;
}

void printLCD(void) {
	lcd_send_cmd(0x80 | 0x09);
//	lcd_send_data(voltage);
	sprintf(s1.str, "%d", voltage);
	lcd_send_string(s1.str);
	lcd_send_cmd(0x80 | 0x49);
	sprintf(s2.str, "%f", current);
	lcd_send_string(s2.str);
//	lcd_send_data(current);
	lcd_send_cmd(0x80 | 0x1C);
	sprintf(s3.str, "%f", lowVal);
	lcd_send_string(s3.str);
//	lcd_send_data(lowVal);

	lcd_send_cmd(0x80 | 0x5C);
	sprintf(s4.str, "%f", highVal);
	lcd_send_string(s4.str);
//	lcd_send_data(lowVal);
	/* if(lowVal < 10)
	 {
	 lcd.setCursor(5,2);
	 lcd.print("0");
	 lcd.setCursor(6,2);
	 lcd.print(lowVal,1);
	 }
	 else
	 {
	 lcd.setCursor(5,2);
	 lcd.print(lowVal,1);
	 }
	 if(highVal < 10)
	 {
	 lcd.setCursor(6,3);
	 lcd.print("0");
	 lcd.setCursor(7,3);
	 lcd.print(highVal,1);
	 }
	 else
	 {
	 lcd.setCursor(6,3);
	 lcd.print(highVal,1);
	 }*/
}

void getPotVal(int lval, int hval) {
	long temp = 0, temp1 = 0;
	lowVal = (((lval) * MAXCURRENT) / 4096.0);
	highVal = (((hval) * MAXCURRENT) / 4096.0);
	temp = lowVal * 10;
	lowVal = (float) temp / 10;
	temp1 = highVal * 10;
	highVal = (float) temp1 / 10;
	rawlowVal = 0;
	rawhighVal = 0;
}

/**
 * @brief  The application entry point.
 * @retval int
 */

void init_sys() {
	HAL_Delay(100);
	voltage = readvoltage();
	if (voltage > MINVOLTAGE && voltage < MAXVOLTAGE) {
		Relay_on;
	} else {
		if (voltage < MINVOLTAGE) {
			lcd_clear();
			lcd_send_cmd(0x80 | 0x00);
			lcd_send_string(" LOW VOLTAGE :");
			sprintf(str1, "%d", voltage);
			lcd_send_string(str1);
			Relay_off;
		} else if (voltage > MAXVOLTAGE) {
			lcd_clear();
			lcd_send_cmd(0x80 | 0x00);
			lcd_send_string(" HIGH VOLTAGE :");
			sprintf(str1, "%d", voltage);
			lcd_send_string(str1);
			Relay_off;
		}
		sysState = 1;
		while (1)
			;

	}
	lcd_clear();
	lcd_send_cmd(0x80 | 0x00);
	lcd_send_string("Voltage:");
	lcd_send_cmd(0x80 | 0x40);
	lcd_send_string("Current:");
	lcd_send_cmd(0x80 | 0x14);
	lcd_send_string("Low:");
	lcd_send_cmd(0x80 | 0x54);
	lcd_send_string("High:");
	initOK = 1;
}

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
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	lcd_init();
	//	lcd_send_cmd(0x80 | 0x00);
	setCursor(0, 0);
	lcd_send_string("ANGULAIR");
	HAL_Delay(1000);
	lcd_clear();

	//	lcd_send_cmd(0x80 | 0x40);
	setCursor(3, 1);
	lcd_send_string("INITIALIZING");
	//	LCD1602_PrintInt(12);
	Relay_on;
//	init_sys();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 20);
		ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, 20);
		ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, 20);
		ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, 20);
		ADC_VAL[3] = HAL_ADC_GetValue(&hadc1);

		HAL_ADC_Stop(&hadc1);

		rawhighVal = ADC_VAL[2];
		getPotVal(rawlowVal, rawhighVal);
		lcd_clear();
		setCursor(3, 1);
		sprintf(str1, "%d", rawhighVal);
		lcd_send_string(str1);
		HAL_Delay(500);
		/*if (sysState == 0) {
		 if (initOK == 1) {
		 adc();
		 rawlowVal = ADC_VAL[0];
		 rawhighVal = ADC_VAL[1];
		 getCurrent(ADC_VAL[2]);
		 getPotVal(rawlowVal, rawhighVal);
		 printLCD();
		 if ((current > 0.1)) {
		 faultMon = 1;
		 }
		 if (faultMon) {
		 if (current < lowVal) {
		 faultCount++;
		 if (faultCount >= 20) {
		 Relay_off;
		 HAL_Delay(2000);
		 fault_on;
		 sysState = 1;
		 }
		 } else if (current > highVal) {
		 faultCount++;
		 if (faultCount % 5 == 0) {
		 Relay_off;
		 HAL_Delay(2000);
		 fault_on;
		 sysState = 1;
		 }
		 } else {
		 faultCount = 0;
		 fault_off;
		 sysState = 0;
		 }
		 }

		 }
		 }*/
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 4;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB0 PB1 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

