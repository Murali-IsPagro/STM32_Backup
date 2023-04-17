/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "stdbool.h"
#include "i2c-lcd.h"
#include "string.h"
#include "stdio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!CAUTION!!!!-----------------CHANGE set of values ONLY HERE*/

#define SET_OF_VALUES 10






/* USER CODE END Includes */
#define START_BUTTON HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) //PD0---->Start Button
#define SET_RESET_BUTTON HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1)//PD1-->Set reset Button
#define SYSTEM_RESET_BUTTON HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2)//PD2-->System reset Button
#define LASER_1_INPUT HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) //PD2 ---> Laser 1 input
#define LASER_2_INPUT HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) // PD3---> Laser 2 input

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t set_counter=0;
uint32_t total_count=0;
uint32_t recheck=0;
/* USER CODE END PTD */
bool state=true;
bool confirm=true;
bool trigger=false;


char MSG[50]={'/0'};
char str0[50]={'/0'};
char str1[50]={'/0'};// string to store counter value of a set (100,50, etc)
char str2[50]={'/0'};//string to store total counter

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1,"hello\n", 10, 100);
  /* USER CODE END 2 */
 // lcd_init();
  	//	lcd_send_cmd(0x80 | 0x00);
  	//setCursor(0, 0);
 // 	lcd_send_string("ANGULAIR");
  lcd_init();

  lcd_send_cmd (0x80|0x00);
  lcd_send_string("HELLO WORLD");

  lcd_send_cmd (0x80|0x40);
  lcd_send_string("LCD 20x4 DEMO");

  lcd_send_cmd (0x80|0x1C);
  lcd_send_string("BY");

  lcd_send_cmd (0x80|0x54);
    lcd_send_string("ControllersTech");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(START_BUTTON==GPIO_PIN_SET){
	 			  trigger=true;
	 			 lcd_clear();
///////////////////////////
	 			 // HAL_Delay(1000);
	 		  }
	 		  if(SYSTEM_RESET_BUTTON==GPIO_PIN_SET){
	 			  HAL_NVIC_SystemReset();
	 		  }
	 	    /* USER CODE END WHILE */
	 		  	  while(trigger && set_counter!=SET_OF_VALUES){
	 		  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

	 		  		  /*Laser1 operation*/
	 		  		  if(LASER_1_INPUT==GPIO_PIN_RESET && state){
	 		  			  set_counter++;
	 		  			  state=false;
	 		  		  }
	 		  		  if(LASER_1_INPUT==GPIO_PIN_SET){
	 		  			  state=true;
	 		  		  }


	 		  		  /*Laser2 operation calculates total count*/
	 		  		  if(LASER_2_INPUT==GPIO_PIN_RESET && confirm){
	 		  			  total_count++;
	 		  			  recheck=set_counter;
	 		  			  confirm=false;
	 		  		  }
	 		  		  if(LASER_2_INPUT==GPIO_PIN_SET){
	 		  			  confirm=true;
	 		  		  }


	 		  		  /*Transmitting data to computer*/
	 		  		  sprintf(MSG," inside SET_value:%d,Total_count=%d \n",set_counter,total_count);
	 		  		  HAL_UART_Transmit(&huart1, MSG, sizeof(MSG), 100);
	 		  		  //HAL_Delay(10);



	 		  		/*----------------Transmitting to LCD Display------------------*/
	 		 	//	lcd_clear();
		 		 		setCursor(0, 0);//(column, row)
		 		 		sprintf(str0, "SET_VALUE:%d", SET_OF_VALUES);
		 		 		lcd_send_string(str0);
	 		 		setCursor(0, 1);//(column, row)
	 		 		sprintf(str1, "Set_count:%d", set_counter);
	 		 		lcd_send_string(str1);
	 		 		setCursor(0, 2);
	 		 		sprintf(str2, "Total_count:%d", total_count);
	 		 		lcd_send_string(str2);
	 		  	  }

	 		  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);//turn off conveyor once the loop exit and waiting for reset button



	 	/*Reset the set_counter value to zero and push the actuator*/
	 		  	if(set_counter==SET_OF_VALUES && SET_RESET_BUTTON==GPIO_PIN_SET){
	 		  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // push the actuator relay
	 		  	set_counter=0;
	 		  	recheck=0;
	 		  	trigger=false;
	 		  	}


 		 		setCursor(0, 0);//(column, row)
 		 		sprintf(str0, "SET_VALUE:%d", SET_OF_VALUES);
 		 		lcd_send_string(str0);
		 		setCursor(0, 1);//(column, row)
		 		sprintf(str1, "Set_count:%d", set_counter);
		 		lcd_send_string(str1);
		 		setCursor(0, 2);
		 		sprintf(str2, "Total_count:%d", total_count);
		 		lcd_send_string(str2);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
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
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

