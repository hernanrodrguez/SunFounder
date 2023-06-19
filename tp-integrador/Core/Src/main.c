/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "keypad.h"
#include "task.h"
#include "lcd_i2c_pcf8574.h"
#include "MPU6050.h"
#include "I2Cdev.h"
#include "queue.h"
#include "math.h"
#include "debounce.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	float AngleRoll;
	float AnglePitch;
}Angle_data;

typedef struct{
	uint8_t AngleValue;
	uint8_t WhichAngle;
}LCD_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Buzzer_On() HAL_GPIO_WritePin(BuzzerGPIO_GPIO_Port, BuzzerGPIO_Pin, GPIO_PIN_SET)
#define Buzzer_Off() HAL_GPIO_WritePin(BuzzerGPIO_GPIO_Port, BuzzerGPIO_Pin, GPIO_PIN_RESET)
#define keyToZero() if(key==11) key = 0
#define ENTER_PITCH 10
#define ENTER_ROLL 	12
#define PITCH_ANGLE_LCD 1
#define ROLL_ANGLE_LCD 2
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

QueueHandle_t queueAngle;
QueueHandle_t queueKey;
QueueHandle_t queueLCD;

extern debounce_t deb_col_1;
extern debounce_t deb_col_2;
extern debounce_t deb_col_3;


//todo sacar public solo para debug
char string[128]; // Sin el static el count del semaforo volaba (raro)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void MPU6050_Task(void *pvParameters) {
	int16_t ax, ay, az;

	float AccXinG,AccYinG,AccZinG;

	Angle_data angle_to_send;


	while (1) {

		MPU6050_getAcceleration(&ax, &ay, &az);

		AccXinG = (float) ax/4096;
		AccYinG = (float) ay/4096;
		AccZinG = (float) az/4096;

		angle_to_send.AngleRoll =atan(AccYinG/sqrt(AccXinG*AccXinG+AccZinG*AccZinG))*1/(3.142/180);
		angle_to_send.AnglePitch =-atan(AccXinG/sqrt(AccYinG*AccYinG+AccZinG*AccZinG))*1/(3.142/180);

		xQueueSendToBack(queueAngle,&angle_to_send,portMAX_DELAY);

	}
}

static void KeyPad_Task(void *pvParameters) {

	static uint8_t key;

	while(1){

		key = read_keypad();
		if(key != 0){
			keyToZero();
			xQueueSendToBack(queueKey,&key,portMAX_DELAY);
		}
	}

}

static void LCD_Print(void *pvParameters) {

	LCD_data lcd_data_keyboard;

	while(1)
	{
		if(pdTRUE == xQueueReceive(queueLCD, &lcd_data_keyboard, 0)){
			lcd_Clearscreen();
			sprintf(string,"%.*d",3,lcd_data_keyboard.AngleValue);
			lcd_SendString(string);
		}

	}

}

static void SunFounder_Task(void *pvParameters) {

	typedef enum {
		STATE_UNIT, STATE_DEC, STATE_CEN, STATE_ENTER,
		STATE_SAVE_PITCH, STATE_SAVE_ROLL
	} State;
	static State currentState = STATE_UNIT;

	LCD_data lcd_data;
	lcd_data.WhichAngle = 0;

	Angle_data angle_read;
	static uint8_t key;
	static uint8_t keyword;
	static uint8_t pitch_angle_set = 45;
	static uint8_t roll_angle_set = 45;

	while (1) {

		xQueueReceive(queueAngle, &angle_read, portMAX_DELAY);

		 if(angle_read.AngleRoll > roll_angle_set - 3 && angle_read.AngleRoll < roll_angle_set + 3){
			 Buzzer_On();
		 }else{
			 Buzzer_Off();
		 }

		if(pdTRUE == xQueueReceive(queueKey, &key, 0)){
			switch (currentState) {
				case STATE_UNIT:
					if(key<10){
						keyword = key;
						lcd_data.AngleValue = keyword;
						lcd_data.WhichAngle = 0;
						xQueueSendToBack(queueLCD,&lcd_data,portMAX_DELAY);
						currentState = STATE_DEC;
					}
					break;

				case STATE_DEC:
					if(key<10){
						keyword = keyword*10 + key;
						lcd_data.AngleValue = keyword;
						lcd_data.WhichAngle = 0;
						xQueueSendToBack(queueLCD,&lcd_data,portMAX_DELAY);
						currentState = STATE_CEN;
					}else if(key == ENTER_ROLL){
						currentState = STATE_SAVE_ROLL;
					}else if(key == ENTER_PITCH){
						currentState = STATE_SAVE_PITCH;
					}else{
						keyword = 0;
						currentState = STATE_UNIT;
					}
					break;

				case STATE_CEN:
					if(key<10){
						keyword = keyword*10 + key;
						lcd_data.AngleValue = keyword;
						lcd_data.WhichAngle = 0;
						xQueueSendToBack(queueLCD,&lcd_data,portMAX_DELAY);
						currentState = STATE_ENTER;
					}else if(key == ENTER_ROLL){
						currentState = STATE_SAVE_ROLL;
					}else if(key == ENTER_PITCH){
						currentState = STATE_SAVE_PITCH;
					}else{
						keyword = 0;
						currentState = STATE_UNIT;
					}
					break;
				case STATE_ENTER:
					if(key<10){
						keyword = 0;
						lcd_data.AngleValue = keyword;
						lcd_data.WhichAngle = 0;
						xQueueSendToBack(queueLCD,&lcd_data,portMAX_DELAY);
						currentState = STATE_UNIT;
					}else if(key == ENTER_ROLL){
						currentState = STATE_SAVE_ROLL;
					}else if(key == ENTER_PITCH){
						currentState = STATE_SAVE_PITCH;
					}else{
						keyword = 0;
						currentState = STATE_UNIT;
					}
					break;

				case STATE_SAVE_ROLL:
					roll_angle_set = keyword;
					lcd_data.AngleValue = keyword;
					lcd_data.WhichAngle = ROLL_ANGLE_LCD;
					xQueueSendToBack(queueLCD,&lcd_data,portMAX_DELAY);
					keyword = 0;
					currentState = STATE_UNIT;
					break;

				case STATE_SAVE_PITCH:
					pitch_angle_set = keyword;
					lcd_data.AngleValue = keyword;
					lcd_data.WhichAngle = PITCH_ANGLE_LCD;
					xQueueSendToBack(queueLCD,&lcd_data,portMAX_DELAY);
					keyword = 0;
					currentState = STATE_UNIT;
					break;

				default:
					keyword = 0;
					currentState = STATE_UNIT;
					break;

			}
		}

	}
}

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  I2Cdev_init(&hi2c1); // init of i2cdevlib.
  MPU6050_initialize();


  //LCD todo
  lcd_Init();
  lcd_SetCursor(0, 0);
  lcd_SendString("Salom");
  lcd_SetCursor(0, 1);
  lcd_SendString("Duniyo!");


  //KeyPad Init
  debounce_init(&deb_col_1, 1, DEBOUNCE_TICKS);
  debounce_init(&deb_col_2, 1, DEBOUNCE_TICKS);
  debounce_init(&deb_col_3, 1, DEBOUNCE_TICKS);

  queueAngle = xQueueCreate(1,sizeof(Angle_data));
  queueKey = xQueueCreate(1,sizeof(uint8_t));

  xTaskCreate(MPU6050_Task,
  			"",
  			configMINIMAL_STACK_SIZE,
  			NULL,
  			2,
  			NULL);

  xTaskCreate(KeyPad_Task,
  			"",
  			configMINIMAL_STACK_SIZE,
  			NULL,
  			2,
  			NULL);

  xTaskCreate(LCD_Print,
  			"",
  			configMINIMAL_STACK_SIZE,
  			NULL,
  			2,
  			NULL);

  xTaskCreate(SunFounder_Task,   			// Nombre de la función que se ejecutará como tarea
  			"",                      	// Nombre de la tarea (cadena vacía)
  			configMINIMAL_STACK_SIZE,	// Tamaño de la pila asignado a la tarea
  			NULL, 						// Puntero a información que se pasará como argumento a la tarea
  			2,                       	// Prioridad de la tarea
  			NULL); 						// Puntero a variable para almacenar el identificador de la tarea


  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ROW_1_Pin|ROW_2_Pin|ROW_3_Pin|ROW_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BuzzerGPIO_GPIO_Port, BuzzerGPIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ROW_1_Pin ROW_2_Pin ROW_3_Pin ROW_4_Pin */
  GPIO_InitStruct.Pin = ROW_1_Pin|ROW_2_Pin|ROW_3_Pin|ROW_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : COL_1_Pin COL_2_Pin COL_3_Pin */
  GPIO_InitStruct.Pin = COL_1_Pin|COL_2_Pin|COL_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BuzzerGPIO_Pin */
  GPIO_InitStruct.Pin = BuzzerGPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BuzzerGPIO_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
