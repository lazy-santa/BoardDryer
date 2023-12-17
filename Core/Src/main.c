

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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lcd1602_i2c_lib.h"
#include "bmp280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TimerMode 1
#define NoTimerMode 2
#define DebugMode 3

#define MinTemperature 25
#define MaxTemperature 50
#define DELTA_TEMP 1

#define MinTime 1
#define MaxTime 90
#define Delta_Time 30
#define Time_Check 30
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char *ListModes[4] = { "Set Mode: ", "1.Witn set time ", "2.Witnout time  ", "3.Diagnostics   " };
uint8_t Info_Mass_Index = 0;

volatile uint8_t ButtonDoor_State = 0;
volatile uint8_t ButtonEncoder_State = 0;
uint8_t past_encValue = 0;

uint8_t TargetMode = TimerMode;
int16_t TargetTemp = MinTemperature;
uint16_t TargetTime = 0;

uint16_t temp_choose = 30;
uint16_t time_choose = 30;

float Count_Timer = 0;
float Count_CheckTimer = 30;

BMP280_HandleTypedef bmp280;
float pressure = 0, temperature = 0;
float past_checkTemp = 0;

extern char lcd1602_tx_buffer[40];
uint8_t State_Lamp = 0, State_Cool = 0, Alarm_Flag = 0;

float temp1 = 0, temp2 = 0;
int flag[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Установка цветов RGB-светодиода (Значения; 0-255)
void LED_RGB_Set(uint8_t LED_Red_Value, uint8_t LED_Green_Value, uint8_t LED_Blue_Value) {
	htim2.Instance->CCR2 = LED_Red_Value; // Red
	htim2.Instance->CCR3 = LED_Green_Value; // Green
	htim2.Instance->CCR4 = LED_Blue_Value; // Blue
}

//Установка звукового сигнала
void BuzzerSet(uint16_t time, uint16_t fill) {
	TIM1->PSC -=10;
	TIM4->CCR1 = TIM4->ARR * (fill/100);
	HAL_Delay(time);
	TIM4->CCR1 = 0;
}

//Вывод сообщения об ошибке
void Message_Alarm(void){
	LED_RGB_Set(255, 0, 0);
	TIM2->CCR2 = TIM2->ARR / 2;
	BuzzerSet(1000, 50);
	lcd1602_Clean();
	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "  Error!  ");
	lcd1602_Print_text(lcd1602_tx_buffer);
//	lcd1602_SetCursor(0, 1);
//	sprintf(lcd1602_tx_buffer, "Run diagnostics");
//	lcd1602_Print_text(lcd1602_tx_buffer);
	HAL_Delay(5000);
}

//Проверка закрытия двери
void CheckDoor(void){
	if(!ButtonDoor_State){
		if(State_Lamp)
			HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin, GPIO_PIN_RESET);
		if(State_Cool)
			TIM3->CCR1 = 0;
		Count_CheckTimer = 30;
		lcd1602_Clean();
		lcd1602_SetCursor(0, 0);
		sprintf(lcd1602_tx_buffer, "Close door, pls ");
		lcd1602_Print_text(lcd1602_tx_buffer);
		while(!ButtonDoor_State) {
			if (Count_CheckTimer < 0) {
				Alarm_Flag = 1;
				Message_Alarm();

			}
		}
	}
}

//Функция диагностики
void CHECK_SYSTEM(void) {
	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, " CHECK SYSTEMS ");
	lcd1602_Print_text(lcd1602_tx_buffer);

	bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
	temp1 = temperature;

	HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin, GPIO_PIN_SET);
	State_Lamp = 1;
	HAL_Delay(Delta_Time * 1000);
	HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin, GPIO_PIN_RESET);
	State_Lamp = 0;

	bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
	temp2 = temperature;
	flag[0] = (temp2 - temp1) > 0 ? 1 : 0;

	temp1 = temp2;

	TIM3->CCR1 = (uint32_t) (50000);
	State_Cool = 1;
	HAL_Delay(Delta_Time * 1000);
	TIM3->CCR1 = 0;
	State_Cool = 0;

	bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
	temp2 = temperature;
	flag[1] = (temp1 - temp2) < 3 ? 1 : 0;

	lcd1602_Clean();

	switch (flag[0]*10+flag[1]) {
		case 0:
			lcd1602_SetCursor(0, 0);
			sprintf(lcd1602_tx_buffer, "Defect sensor T.");
			lcd1602_Print_text(lcd1602_tx_buffer);
			lcd1602_SetCursor(0, 1);
			sprintf(lcd1602_tx_buffer, "or regul. system");
			lcd1602_Print_text(lcd1602_tx_buffer);
			break;
		case 1:
			lcd1602_SetCursor(0, 0);
			sprintf(lcd1602_tx_buffer, "Defect");
			lcd1602_Print_text(lcd1602_tx_buffer);
			lcd1602_SetCursor(0, 1);
			sprintf(lcd1602_tx_buffer, "heating system");
			lcd1602_Print_text(lcd1602_tx_buffer);
			break;
		case 10:
			lcd1602_SetCursor(0, 0);
			sprintf(lcd1602_tx_buffer, "Defect");
			lcd1602_Print_text(lcd1602_tx_buffer);
			lcd1602_SetCursor(0, 1);
			sprintf(lcd1602_tx_buffer, "cooling system");
			lcd1602_Print_text(lcd1602_tx_buffer);
			break;
		case 11:
			lcd1602_SetCursor(0, 0);
			sprintf(lcd1602_tx_buffer, "System is work");
			lcd1602_Print_text(lcd1602_tx_buffer);
			lcd1602_SetCursor(0, 1);
			sprintf(lcd1602_tx_buffer, "normal:)");
			lcd1602_Print_text(lcd1602_tx_buffer);
			break;
		}
	HAL_Delay(10000);
	ButtonEncoder_State = 0;
	lcd1602_Clean();
}

// Установка целевого режима
uint8_t ChooseTargetMode() {
	LED_RGB_Set(255, 255, 0);	//Установка индикации светодиода: Желтый цвет

	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, ListModes[0]);
	lcd1602_Print_text(lcd1602_tx_buffer);

	while (!ButtonEncoder_State) {
		Info_Mass_Index = TIM1->CNT;
		lcd1602_SetCursor(0, 1);
		sprintf(lcd1602_tx_buffer, ListModes[Info_Mass_Index % 3 + 1]);
		lcd1602_Print_text(lcd1602_tx_buffer);
	}

	ButtonEncoder_State = 0;
	lcd1602_Clean();
	return (Info_Mass_Index % 3 + 1);
}

// Установка времени работы
uint8_t ChooseTargetTemp() {
	LED_RGB_Set(255, 255, 0);	//Установка индикации светодиода: Желтый цвет

	while (!ButtonEncoder_State) {
		lcd1602_SetCursor(0, 0);
		sprintf(lcd1602_tx_buffer, "Set T: %d*C  ", temp_choose);
		lcd1602_Print_text(lcd1602_tx_buffer);

		if(TIM1->CNT > past_encValue){
			if (temp_choose + 1 > MaxTemperature)
				temp_choose = MaxTemperature;
			else
					temp_choose++;
		}
		if(TIM1->CNT < past_encValue){
			if (temp_choose - 1 < MinTemperature)
				temp_choose = MinTemperature;
			else
				temp_choose--;
		}
		past_encValue = TIM1->CNT;

	}
	ButtonEncoder_State = 0;
	TIM1->CNT &= 0x0;
	lcd1602_Clean();
	return temp_choose;
}

// Установка температуры работы
uint8_t ChooseTargetTime() {
	LED_RGB_Set(255, 255, 0);	//Установка индикации светодиода: Желтый цвет

	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "Set time: ");
	lcd1602_Print_text(lcd1602_tx_buffer);

	while (!ButtonEncoder_State) {
		lcd1602_SetCursor(0, 1);
		sprintf(lcd1602_tx_buffer, "%d H. %d min.  ",
				time_choose / 60,
				time_choose % 60);
		lcd1602_Print_text(lcd1602_tx_buffer);

		if(TIM1->CNT > past_encValue){
			if (time_choose + 1 > MaxTime)
				time_choose = MaxTime;
			else
				time_choose++;
		}
		if(TIM1->CNT < past_encValue){
			if (time_choose - 1 < MinTime)
				time_choose = MinTime;
			else
				time_choose--;
		}
		past_encValue = TIM1->CNT;
	}
	ButtonEncoder_State = 0;
	TIM1->CNT &= 0x0;
	lcd1602_Clean();
	return time_choose;
}

//Нагрев перед работой
void Heating(int16_t TargetTemp){
	HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin, GPIO_PIN_SET);
	State_Lamp = 1;

	while (temperature <= TargetTemp) {
		bmp280_read_float(&bmp280, &temperature, &pressure, NULL);

		lcd1602_SetCursor(0, 0);
		sprintf(lcd1602_tx_buffer, "Temperature: %d ",
				(uint8_t) temperature);
		lcd1602_Print_text(lcd1602_tx_buffer);
		CheckDoor();
		if(Alarm_Flag) break;
	}
	HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin, GPIO_PIN_RESET);
	State_Lamp = 0;
	lcd1602_Clean();
}

//Режим по Таймеру
void RunTimerMode(int16_t TargetTemp, uint16_t TargetTime){
	LED_RGB_Set(255, 0, 0);
	BuzzerSet(3000, 100);

	Heating(TargetTemp);

	Count_Timer = (float) TargetTime * 60.0;
	Count_CheckTimer = (float) Time_Check * 30;
	past_checkTemp = bmp280_read_float(&bmp280, &temperature, &pressure, NULL);

	while (Count_Timer > 0) {
		bmp280_read_float(&bmp280, &temperature, &pressure, NULL);

		lcd1602_SetCursor(0, 0);
		sprintf(lcd1602_tx_buffer, "Temperature: %d ",
				(uint8_t) temperature);
		lcd1602_Print_text(lcd1602_tx_buffer);

		lcd1602_SetCursor(0, 1);
		sprintf(lcd1602_tx_buffer, "Time: %dH. %dm. ",
				((uint8_t) (Count_Timer / 60) / 60),
				((uint8_t) (Count_Timer / 60) % 60));
		lcd1602_Print_text(lcd1602_tx_buffer);

		if(State_Lamp){
			if (temperature - TargetTemp > DELTA_TEMP) {
				HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin,GPIO_PIN_RESET);
				State_Lamp = 0;
				TIM3->CCR1 = (uint32_t) (50000);
				State_Cool = 1;
			}
		}
		else{
			if (TargetTemp - temperature > DELTA_TEMP) {
				HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin, GPIO_PIN_SET);
				State_Lamp = 1;
				TIM3->CCR1 = 0;
				State_Cool = 0;
			}
		}
		CheckDoor();
		if(Alarm_Flag) break;
	}
}

//Режим без таймера (пока не нажата кнопка)
void RunNoTimerMode(int16_t TargetTemp){
	LED_RGB_Set(255, 0, 0);
	BuzzerSet(3000, 100);

	Heating(TargetTemp);

	ButtonEncoder_State = 0;
	while (!ButtonEncoder_State) {

		bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
		lcd1602_SetCursor(0, 0);
		sprintf(lcd1602_tx_buffer, "    <Working>   ");
		lcd1602_Print_text(lcd1602_tx_buffer);
		lcd1602_SetCursor(0, 1);
		sprintf(lcd1602_tx_buffer, "Temperature: %d ",
				(uint8_t) temperature);
		lcd1602_Print_text(lcd1602_tx_buffer);
		HAL_Delay(50);

		if(State_Lamp){
			if (temperature - TargetTemp > DELTA_TEMP) {
				HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin,GPIO_PIN_RESET);
				State_Lamp = 0;
				TIM3->CCR1 = (uint32_t) (50000);
				State_Cool = 1;
			}
		}
		else{
			if (TargetTemp - temperature > DELTA_TEMP) {
				HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin, GPIO_PIN_SET);
				State_Lamp = 1;
				TIM3->CCR1 = 0;
				State_Cool = 0;
			}
		}
		CheckDoor();
		if(Alarm_Flag) break;
	}
	ButtonEncoder_State = 0;
	lcd1602_Clean();
}

//Окончание работы, охлаждение
void FinishWork(){
	lcd1602_Clean();
	HAL_GPIO_WritePin(LAMPS_GPIO_Port, LAMPS_Pin, GPIO_PIN_RESET);
	State_Lamp = 0;
	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "Work is finished");
	lcd1602_Print_text(lcd1602_tx_buffer);
	
	LED_RGB_Set(0, 255, 0);
	ButtonEncoder_State = 0;
	
	State_Cool = 1;
	for(int i = 0; i < 3; i++){
		Count_Timer = 30000;
		BuzzerSet(3000, 100);
		while (Count_Timer > 0){
			bmp280_read_float(&bmp280, &temperature, &pressure, NULL);
			TIM3->CCR1 = 50000 * (temperature/TargetTemp);
	
			if (ButtonEncoder_State) break;
		}
		if (ButtonEncoder_State) break;
	}
	TIM3->CCR1 = 0;
	State_Cool = 0;
	ButtonEncoder_State = 0;
	lcd1602_Clean();
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  	HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL);

  	HAL_TIM_Base_Start_IT(&htim2);
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  	HAL_TIM_Base_Start_IT(&htim3);
  	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  	HAL_TIM_Base_Start_IT(&htim4);
  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  	lcd1602_Init();

	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "     Check      ");
	lcd1602_Print_text(lcd1602_tx_buffer);
	lcd1602_SetCursor(0, 1);
	sprintf(lcd1602_tx_buffer, "   Temp sensor  ");
	lcd1602_Print_text(lcd1602_tx_buffer);

	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while (!(bmp280_init(&bmp280, &bmp280.params)))
	{
	 HAL_Delay(1000);
	}
	//CHECK_SYSTEM();
	lcd1602_Clean();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
  	bmp280_read_float(&bmp280, &temperature, &pressure, NULL);

	lcd1602_SetCursor(0, 0);
	sprintf(lcd1602_tx_buffer, "Temperature: %d ",
			(uint8_t) temperature);
	lcd1602_Print_text(lcd1602_tx_buffer);

	lcd1602_SetCursor(0, 1);
	sprintf(lcd1602_tx_buffer, "Time: %dH. %dm. ",
			((uint8_t) (Count_Timer / 60) / 60),
			((uint8_t) (Count_Timer / 60) % 60));
	lcd1602_Print_text(lcd1602_tx_buffer);
  	ButtonDoor_State = HAL_GPIO_ReadPin(BUTTON_DOOR_GPIO_Port, BUTTON_DOOR_Pin);
	TargetMode = ChooseTargetMode();

	switch (TargetMode) {
	case TimerMode:
		TargetTime = ChooseTargetTime();
		TargetTemp = ChooseTargetTemp();
		CheckDoor();
		RunTimerMode(TargetTemp, TargetTime);
		if(Alarm_Flag)
		break;
		FinishWork();
		break;

	case NoTimerMode:
		TargetTemp = ChooseTargetTemp();
		CheckDoor();
		RunNoTimerMode(TargetTemp);
		if(Alarm_Flag)
		break;
		FinishWork();
		break;

	case DebugMode:
		LED_RGB_Set(0, 0, 255);
		BuzzerSet(1000, 100);
		CHECK_SYSTEM();
		break;
	}
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
