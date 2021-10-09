/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "ST7920_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct Button
{
	char* Label;
	GPIO_TypeDef* GPIO;
	uint16_t GPIO_Pin;
	GPIO_TypeDef* GPIO_Out;
	uint16_t GPIO_Pin_Out;
	uint8_t B_counter;
	uint8_t B_State;
	uint8_t B_Out;
	uint8_t Lock;
	uint8_t Mode;
	uint16_t Delay;
	struct Button* addiction;
	void (*call_function)(struct Button *button);
	void (*alternate_function)(struct Button *button);
	void (*third_function)(uint32_t* steps);
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HOLD 0
#define PRESS 1
#define TOGGLE 2
#define TIMER 3
#define __DELAY 4
#define HOLD_MOTOR 5
#define HOLD_UNTIL 6

#define STEPS 12000
#define ACCEL 800

#define PULL_STEPS 1720
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

// Переменные для работы счётчика (принцип энкодера)

uint8_t OutSignals[][2] = {{0,0},{1,0},{1,1},{0,1}};
uint8_t *pOutSig = OutSignals[0];

extern char tx_buffer[128]; //Буфер для отправки текста на дисплей
extern uint8_t Frame_buffer[1024]; //Буфер кадра

static uint32_t Pullsteps = PULL_STEPS, Dosesteps = STEPS;
uint32_t counter = 0, count_prev = 0;

char tmpflg = 0;

static bool flag = false;
#define WELD_DELAY 600
#define BUTTONS_COUNT 23
struct Button Buttons[BUTTONS_COUNT];
static uint32_t Defines[BUTTONS_COUNT][6] = {
		{(uint32_t)V_Push_V_Weld_GPIO_Port, V_Push_V_Weld_Pin, 0, 0, PRESS, 0},															// 0Macros
		{(uint32_t)Prepare_material_GPIO_Port, Prepare_material_Pin, (uint32_t)0, 0, HOLD_UNTIL, 12000},								// 1Prepare material from bobbin
		{(uint32_t)Dose_GPIO_Port, Dose_Pin, (uint32_t)Dose_Out_GPIO_Port, Dose_Out_Pin, TIMER, 2580},									// 2Dose motor control
		{(uint32_t)V_Push_GPIO_Port, V_Push_Pin, (uint32_t)V_Push_Out_GPIO_Port, V_Push_Out_Pin, TOGGLE, 0},							// 3Vertical cylinder push
		{(uint32_t)V_Weld_GPIO_Port, V_Weld_Pin, (uint32_t)V_Weld_Out_GPIO_Port, V_Weld_Out_Pin, __DELAY, WELD_DELAY},					// 4Vertical Weld spot
		{(uint32_t)H_Push_GPIO_Port, H_Push_Pin, (uint32_t)H_Push_Out_GPIO_Port, H_Push_Out_Pin, TOGGLE, 0},							// 5Horizontal cylinder push
		{(uint32_t)H_Weld_GPIO_Port, H_Weld_Pin, (uint32_t)H_Weld_Out_GPIO_Port, H_Weld_Out_Pin, __DELAY, WELD_DELAY},					// 6Horizontal Weld spot
		{(uint32_t)Pull_GPIO_Port, Pull_Pin, (uint32_t)Pull_Out_GPIO_Port, Pull_Out_Pin, TIMER, 2600},									// 7Pull material down
		{(uint32_t)Cut_GPIO_Port, Cut_Pin, (uint32_t)Cut_Out_GPIO_Port, Cut_Out_Pin, HOLD, 0},											// 8Cutter
		{(uint32_t)Mode_GPIO_Port, Mode_Pin, 0, 0, HOLD, 0},																			// 9Mode select
		{(uint32_t)Auto_Start_GPIO_Port, Auto_Start_Pin, 0, 0, TOGGLE, 0},																// 10Auto mode start
		{(uint32_t)HOLD_Dose_GPIO_Port, HOLD_Dose_Pin, (uint32_t)Dose_Out_GPIO_Port, Dose_Out_Pin, HOLD_MOTOR, 2580},					// 11HOLD_Dose motor
		{(uint32_t)HOLD_Pull_GPIO_Port, HOLD_Pull_Pin, (uint32_t)Pull_Out_GPIO_Port, Pull_Out_Pin, HOLD_MOTOR, 2580},					// 12HOLD_Pull motor
		{(uint32_t)Reed_Switch_GPIO_Port, Reed_Switch_Pin, (uint32_t)0, 0, 200, 2580},													// 13Reed Switch feedback
		{(uint32_t)CounterReset_GPIO_Port, CounterReset_Pin, (uint32_t)0, 0, 0, 0},														// 14Counter reset
		{(uint32_t)Size_Select_GPIO_Port, Size_Select_Pin, (uint32_t)0, 0, 0, 0},														// 15Pocket size select
		{(uint32_t)Dose_GPIO_Port, Dose_Pin, (uint32_t)Dose_Pulse_Out_GPIO_Port, Dose_Pulse_Out_Pin, __DELAY, 100},						// 16Pulse out for dose machine
		{(uint32_t)Dose_Ready_GPIO_Port, Dose_Ready_Pin, (uint32_t)0, 0, 0, 0},															// 17Dose feedback signal (When ready)
		{(uint32_t)Weld_Feedback_GPIO_Port, Weld_Feedback_Pin, (uint32_t)0, 0, 0, 0},													// 18Weld feedback (Check welder error)
		{(uint32_t)0, 0, (uint32_t)Buzzer_GPIO_Port, Buzzer_Pin, 0, 0},																	// 19Buzzer
		{(uint32_t)Weld_Feedback2_GPIO_Port, Weld_Feedback2_Pin, (uint32_t)0, 0, 0, 0},													// 20Weld feedback2 (Check welder error)
		{(uint32_t)VPush_FeedBack_GPIO_Port, VPush_FeedBack_Pin, (uint32_t)0, 0, 0, 0},													// 21VPush_FeedBack
		{(uint32_t)HorizontalPistonFeedback_GPIO_Port, HorizontalPistonFeedback_Pin, (uint32_t)0, 0, 0, 0}								// 22HPush_FeedBack
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Init(void);
void toggle_func(struct Button *button);
void macros1(struct Button *button);
void Delay(uint32_t delay);
void SetSteps0(uint32_t* steps);
void SetSteps1(uint32_t* steps);
void SetSteps2(uint32_t* steps);
void TimerMotor(struct Button *Button);
void HoldMotor(struct Button *Button, uint8_t mode);
void HoldPrepareMotorUntill(struct Button *Button, uint8_t mode);
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
  MX_TIM7_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //Reset output pins
  GPIOD->ODR = ~0;
  Init();
  HAL_Delay(2300);
  //Engage spi display
  //ST7920_Clean();
  //sprintf(tx_buffer, "Count: %lu", counter);
  //ST7920_Decode_UTF8(20, 4, 0, tx_buffer);
  //Engage timers ( buttons check, etc. )
  HAL_TIM_Base_Start_IT(&htim7);
  //HAL_TIM_Base_Start_IT(&htim6);
  HAL_GPIO_WritePin(OUT_A_GPIO_Port, OUT_A_Pin, 1);
  HAL_GPIO_WritePin(OUT_B_GPIO_Port, OUT_B_Pin, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{

		if(!Buttons[9].B_State)// Mode select
		{
			tmpflg = 1;
			if(Buttons[9].B_Out == 1) // Reset buttons and timers
			{
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
				Buttons[9].B_Out = 0;
				for(uint8_t i = 0; i < BUTTONS_COUNT; i++)
				{
					Buttons[i].B_Out = 0;
				};
			}
			// MANUAL MODE
			for(uint8_t i = 0; i < BUTTONS_COUNT; i++)
			{
				if(Buttons[i].Lock == 0)
				{
					if(Buttons[i].Mode == HOLD)
					{
						HAL_GPIO_WritePin(Buttons[i].GPIO_Out, Buttons[i].GPIO_Pin_Out, !Buttons[i].B_State);
						continue;
					}
					if(Buttons[i].Mode == TOGGLE && Buttons[i].alternate_function == 0)
					{
						if(Buttons[i].addiction != 0)
							HAL_GPIO_WritePin(Buttons[i].GPIO_Out, Buttons[i].GPIO_Pin_Out, !(Buttons[i].B_Out && Buttons[i].addiction->B_Out));
						else
							HAL_GPIO_WritePin(Buttons[i].GPIO_Out, Buttons[i].GPIO_Pin_Out, !Buttons[i].B_Out);
						continue;
					}
					if (Buttons[i].alternate_function && Buttons[i].Mode == PRESS && Buttons[i].B_Out)
					{
						Buttons[i].B_Out = 0;
						Buttons[i].alternate_function(&Buttons[i]);
						continue;
					}
					if(Buttons[i].Mode == TIMER && Buttons[i].B_Out)
					{
						Buttons[i].B_Out = 0;
						TimerMotor(&Buttons[i]);
						continue;
					}
					if(Buttons[i].Mode == __DELAY && Buttons[i].B_Out)
					{
						if(Buttons[i].addiction->B_Out == 0) continue;
						Buttons[i].B_Out = 0;
						HAL_GPIO_WritePin(Buttons[i].GPIO_Out, Buttons[i].GPIO_Pin_Out, 0);
						HAL_Delay(Buttons[i].Delay);
						HAL_GPIO_WritePin(Buttons[i].GPIO_Out, Buttons[i].GPIO_Pin_Out, 1);
						continue;
					}
					if(Buttons[i].Mode == HOLD_MOTOR && Buttons[i].B_Out)
					{
						Buttons[i].B_Out = 0;
						HoldMotor(&Buttons[i], 1);
						continue;
					}
					if(Buttons[i].Mode == HOLD_UNTIL && Buttons[i].B_Out)
					{
						HoldPrepareMotorUntill(Buttons[i].addiction, 1);
						//Buttons[i].B_Out = 0;
						continue;
					}
				}
				if(Buttons[i].Mode == 200) {Buttons[i].B_Out = 0;}
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		else
		{
			if(tmpflg)
			{
				tmpflg = 0;
				Buttons[10].B_Out = 0;
				Buttons[17].B_Out = 0;
			}
			//AUTO MODE
			#define DOSE_PULSE_START HAL_GPIO_WritePin(Buttons[16].GPIO_Out, Buttons[16].GPIO_Pin_Out, 0); // Pulse out when dose
			#define DOSE_PULSE_STOP HAL_GPIO_WritePin(Buttons[16].GPIO_Out, Buttons[16].GPIO_Pin_Out, 1); // Pulse out when dose
			#define PUSH_V HAL_GPIO_WritePin(Buttons[3].GPIO_Out, Buttons[3].GPIO_Pin_Out, 0); // Push
			#define PUSH_H HAL_GPIO_WritePin(Buttons[5].GPIO_Out, Buttons[5].GPIO_Pin_Out, 0); // Push
			#define RELEASE_V HAL_GPIO_WritePin(Buttons[3].GPIO_Out, Buttons[3].GPIO_Pin_Out, 1); //Release
			#define RELEASE_H HAL_GPIO_WritePin(Buttons[5].GPIO_Out, Buttons[5].GPIO_Pin_Out, 1); //Release
			#define CUT_START HAL_GPIO_WritePin(Buttons[8].GPIO_Out, Buttons[8].GPIO_Pin_Out, 0); // CUT
			#define CUT_RELEASE HAL_GPIO_WritePin(Buttons[8].GPIO_Out, Buttons[8].GPIO_Pin_Out, 1); //Release
			#define WELD_V_START HAL_GPIO_WritePin(Buttons[4].GPIO_Out, Buttons[4].GPIO_Pin_Out, 0); // Weld
			#define WELD_H_START HAL_GPIO_WritePin(Buttons[6].GPIO_Out, Buttons[6].GPIO_Pin_Out, 0); // Weld
			#define WELD_V_STOP HAL_GPIO_WritePin(Buttons[4].GPIO_Out, Buttons[4].GPIO_Pin_Out, 1);
			#define WELD_H_STOP HAL_GPIO_WritePin(Buttons[6].GPIO_Out, Buttons[6].GPIO_Pin_Out, 1);
			#define PULL TimerMotor(&Buttons[7]); // PULL mat.
			#define DOSE TimerMotor(&Buttons[2]); // Dose
			#define CYCLE_DELAY HAL_Delay(100);
			#define WELD_TIME HAL_Delay(600);
			#define WAITING 450

			#define VPISTON_STATE Buttons[21].B_State
			#define HPISTON_STATE Buttons[22].B_State
			void WaitPistons(void)
			{
				uint8_t timer = 0;
				while(VPISTON_STATE == 0 || HPISTON_STATE == 0)
				{
					timer++;
					HAL_Delay(10);
					if(timer >= 10)
					{
						Buttons[10].B_Out = 0;
						timer = 0;
						while(!Buttons[10].B_Out && !Buttons[21].B_State && !Buttons[22].B_State)
						{
							HAL_GPIO_WritePin(Buttons[19].GPIO_Out, Buttons[6].GPIO_Pin_Out, 0);
							HAL_Delay(200);
							HAL_GPIO_WritePin(Buttons[19].GPIO_Out, Buttons[6].GPIO_Pin_Out, 1);
							HAL_Delay(200);
						};
					}
				}
			}
			#define WaitPush WaitPistons();

			void WaitDoseReady(void)
			{
				while(Buttons[17].B_Out)
				{
					asm("NOP");
					if(!Buttons[10].B_Out || !Buttons[9].B_State)
					{
						break;
					};
					/*if(Buttons[17].B_Out && !Buttons[17].B_State)
					{
						break;
					}
					*/
				};
				Buttons[17].B_Out = 1;
			};
			#define WAIT_DOSE_READY WaitDoseReady();
			void Welder_Check(void)
			{
				uint32_t timer = 12000000;
				while(!Buttons[18].B_State && !Buttons[20].B_State)
				{
					timer--;
					if(!timer)
					{
						WELD_H_STOP
						WELD_V_STOP
						Buttons[10].B_Out = 0;
						while(!Buttons[10].B_Out && (!Buttons[18].B_State || !Buttons[20].B_State))
						{
							HAL_GPIO_WritePin(Buttons[19].GPIO_Out, Buttons[6].GPIO_Pin_Out, 0);
							HAL_Delay(200);
							HAL_GPIO_WritePin(Buttons[19].GPIO_Out, Buttons[6].GPIO_Pin_Out, 1);
							HAL_Delay(200);
						};
						WELD_H_START
						WELD_V_START
						break;
					};
				};

			}
			#define WELD_CHECK Welder_Check();

			if(Buttons[10].B_Out) // AUTO MODE START
			{
				if(Buttons[15].B_State == 1)
				{
					Pullsteps = PULL_STEPS / 2;
					Dosesteps = STEPS;
					//Pull new material
					PULL
					CYCLE_DELAY
					//Clamp material 1, cut pocket
					PUSH_H
					PUSH_V
					CUT_START
					CYCLE_DELAY
					CUT_RELEASE
					//Welding stage 1
					WELD_H_START
					WELD_V_START
					WELD_CHECK
					WELD_TIME
					WELD_H_STOP
					WELD_V_STOP
					//Release
					RELEASE_H
					RELEASE_V
					CYCLE_DELAY
					//Fill, release
					WAIT_DOSE_READY
					DOSE_PULSE_START
					CYCLE_DELAY
					DOSE_PULSE_STOP
					HAL_Delay(WAITING); // Delay. Mixture fall time
					CYCLE_DELAY
				}
				else
				{
					Pullsteps = PULL_STEPS / 2;
					Dosesteps = STEPS;
					//Pull new material
					PULL
					CYCLE_DELAY
					//Clamp material 1, cut pocket
					PUSH_H
					PUSH_V
					CUT_START
					CYCLE_DELAY
					CUT_RELEASE
					//Welding stage 1
					WELD_H_START
					WELD_V_START
					WELD_CHECK
					WELD_TIME
					WELD_H_STOP
					WELD_V_STOP
					//Release, fill 1
					RELEASE_H
					RELEASE_V
					WAIT_DOSE_READY
					DOSE_PULSE_START
					CYCLE_DELAY
					DOSE_PULSE_STOP
					CYCLE_DELAY
					HAL_Delay(WAITING); // TEST
					CYCLE_DELAY
					//DOSE
					//Pull stage 2
					PULL
					CYCLE_DELAY
					//Clamp material 2
					PUSH_V
					PUSH_H
					CYCLE_DELAY
					//Welding stage 2
					WELD_V_START
					WELD_H_START
					WELD_CHECK
					WELD_TIME
					WELD_V_STOP
					WELD_H_STOP
					//Release, fill 2
					RELEASE_V
					RELEASE_H
					WAIT_DOSE_READY
					DOSE_PULSE_START
					CYCLE_DELAY
					DOSE_PULSE_STOP
					//DOSE
					CYCLE_DELAY
					HAL_Delay(WAITING); // TEST
					CYCLE_DELAY
					/*
					PULL
					CYCLE_DELAY
					PUSH_H
					PUSH_V
					CYCLE_DELAY
					WELD_H_START
					WELD_V_START
					WELD_TIME
					WELD_H_STOP
					WELD_V_STOP
					DOSE
					PULL
					CYCLE_DELAY
					PUSH_H
					PUSH_V
					CYCLE_DELAY
					WELD_H_START
					WELD_V_START
					WELD_TIME
					WELD_H_STOP
					WELD_V_STOP
					DOSE
					CUT_START
					CYCLE_DELAY
					CUT_RELEASE
					RELEASE_H
					RELEASE_V
					CYCLE_DELAY
					*/
				}
				// Section for quadrature counter
				/*
				static uint8_t mem = 0;
				pOutSig++;
				pOutSig++;
				mem++;
				HAL_Delay(200);
				if(mem > 3)
				{
					pOutSig = OutSignals[0];
					mem = 0;
				}
				HAL_GPIO_WritePin(OUT_A_GPIO_Port, OUT_A_Pin, !pOutSig[0]);
				HAL_GPIO_WritePin(OUT_B_GPIO_Port, OUT_B_Pin, !pOutSig[1]);
				*/
				//End

				counter++; // Ammount of packages done
			}
			else if(flag == true)
			{
				flag = false;
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			}
			//Spi 128x64 screen
			/*
			sprintf(tx_buffer, "Count: %lu", counter);
			ST7920_Decode_UTF8(20, 4, 0, tx_buffer);
			ST7920_Update();
			*/
		}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 150;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 116;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 58;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 24000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 5;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OUT_B_Pin|CS_Pin|OUT_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led_1_Pin|Led_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Dose_Pulse_Out_Pin|Dose_Out_Pin|V_Push_Out_Pin|V_Weld_Out_Pin 
                          |H_Push_Out_Pin|H_Weld_Out_Pin|Pull_Out_Pin|Cut_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OUT_B_Pin OUT_A_Pin */
  GPIO_InitStruct.Pin = OUT_B_Pin|OUT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_2_Pin Button_1_Pin CounterReset_Pin V_Push_V_Weld_Pin 
                           Prepare_material_Pin Dose_Pin V_Push_Pin V_Weld_Pin 
                           H_Push_Pin H_Weld_Pin Pull_Pin Cut_Pin 
                           HOLD_Pull_Pin */
  GPIO_InitStruct.Pin = Button_2_Pin|Button_1_Pin|CounterReset_Pin|V_Push_V_Weld_Pin 
                          |Prepare_material_Pin|Dose_Pin|V_Push_Pin|V_Weld_Pin 
                          |H_Push_Pin|H_Weld_Pin|Pull_Pin|Cut_Pin 
                          |HOLD_Pull_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HOLD_Dose_Pin Weld_Feedback_Pin Size_Select_Pin */
  GPIO_InitStruct.Pin = HOLD_Dose_Pin|Weld_Feedback_Pin|Size_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_1_Pin Led_2_Pin */
  GPIO_InitStruct.Pin = Led_1_Pin|Led_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Dose_Ready_Pin */
  GPIO_InitStruct.Pin = Dose_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Dose_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Weld_Feedback2_Pin Mode_Pin Auto_Start_Pin */
  GPIO_InitStruct.Pin = Weld_Feedback2_Pin|Mode_Pin|Auto_Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Dose_Pulse_Out_Pin Dose_Out_Pin V_Push_Out_Pin V_Weld_Out_Pin 
                           H_Push_Out_Pin H_Weld_Out_Pin Pull_Out_Pin Cut_Out_Pin */
  GPIO_InitStruct.Pin = Dose_Pulse_Out_Pin|Dose_Out_Pin|V_Push_Out_Pin|V_Weld_Out_Pin 
                          |H_Push_Out_Pin|H_Weld_Out_Pin|Pull_Out_Pin|Cut_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Reed_Switch_Pin STOP_Pin */
  GPIO_InitStruct.Pin = Reed_Switch_Pin|STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Init(void)
{
	  HAL_Delay(100);
	  //ST7920_Init();
	  //ST7920_Graphic_mode(1);
	  //sprintf(tx_buffer, "WELCOME");
	  //ST7920_Decode_UTF8(50, 3, 0, tx_buffer);
	  //ST7920_Update();
	  for(uint8_t i = 0; i < BUTTONS_COUNT; i++)
	  {
		  Buttons[i].B_Out = 0;
		  Buttons[i].B_State = 0;
		  Buttons[i].B_counter = 0;
		  Buttons[i].Lock = 0;
		  Buttons[i].call_function = toggle_func;
		  Buttons[i].alternate_function = 0;
		  Buttons[i].GPIO = (GPIO_TypeDef*)Defines[i][0];
		  Buttons[i].GPIO_Pin = Defines[i][1];
		  Buttons[i].GPIO_Out = (GPIO_TypeDef*)Defines[i][2];
		  Buttons[i].GPIO_Pin_Out = Defines[i][3];
		  Buttons[i].Mode = Defines[i][4];
		  Buttons[i].Delay = Defines[i][5];
		  Buttons[i].addiction = 0;
	  }
	  Buttons[0].alternate_function = macros1;
	  Buttons[1].Label = "Prepare";
	  Buttons[1].third_function = SetSteps0;
	  Buttons[1].addiction = &Buttons[13];
	  Buttons[2].third_function = SetSteps2;
	  Buttons[4].addiction = &Buttons[3];
	  Buttons[6].addiction = &Buttons[5];
	  Buttons[7].third_function = SetSteps1;
	  Buttons[9].Label = "mode";
	  Buttons[10].Label = "auto_start";
	  Buttons[13].Label = "reed_switch";
	  HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, 1);
	  HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM7)
	{
		if(HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin) == 0)
		{
			for(uint8_t i = 0; i < BUTTONS_COUNT; i++)
			{
				HAL_GPIO_WritePin(Buttons[i].GPIO_Out, Buttons[i].GPIO_Pin_Out, 1);
			}
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim7);
			Delay(4800000);
			while(HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin) == 0){asm("NOP");};
			HAL_NVIC_SystemReset();
		}

		for(uint8_t i = 0; i < BUTTONS_COUNT; i++)
		{
			if((Buttons[i].addiction->B_Out == 1 && Buttons[i].Mode == __DELAY && Buttons[i].addiction != 0) || Buttons[i].addiction == 0 || Buttons[i].Mode != __DELAY)
				Buttons[i].call_function(&Buttons[i]);
		}
		if(flag == true || Buttons[10].B_Out == 1)
		{
			HoldPrepareMotorUntill(Buttons[1].addiction, 1);
		}
		if((flag == true && Buttons[10].B_Out == 0) || Buttons[9].B_State == 0)
		{
			flag = false;
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		}
		if(Buttons[14].B_Out && counter > 0)
		{
			Buttons[14].B_Out = 0;
			counter = 0;
			//ST7920_Clean();
		}

	}
	if (htim->Instance==TIM6)
	{
		//ST7920_Update();
	}
}

/*
uint8_t find_button(uint8_t* label) // Unused
{
	uint8_t* ptr = label;
	uint8_t length = 0;
	uint8_t i;
	uint8_t j;
	while(ptr++) length++;
	for(i = 0; i < BUTTONS_COUNT; i++)
	{
		for(j = 0; j < length; j++)
		{
			if(label[j] == Buttons[i].Label[j]){if( j==length-1 ){return i;}}
			else{break;};
		}
	}
	return -1;
}
*/
void toggle_func(struct Button *button)
{
	if(HAL_GPIO_ReadPin(button->GPIO, button->GPIO_Pin) == 0 && button->Lock == 0)
		{
			if(button->B_counter < 10)
				button->B_counter++;
			else
				if(button->B_State == 0)
				{
					button->B_State = 1;
					button->B_Out ^= 1;
				}
		}
		else
		{
			if(button->B_counter > 0)
				button->B_counter--;
			else
				button->B_State = 0;
		}
}
void macros1(struct Button *button)
{
		HAL_GPIO_WritePin(Buttons[3].GPIO_Out, Buttons[3].GPIO_Pin_Out, 0);
		HAL_GPIO_WritePin(Buttons[4].GPIO_Out, Buttons[5].GPIO_Pin_Out, 0);
		HAL_Delay(200);
		HAL_GPIO_WritePin(Buttons[5].GPIO_Out, Buttons[4].GPIO_Pin_Out, 0);
		HAL_GPIO_WritePin(Buttons[6].GPIO_Out, Buttons[6].GPIO_Pin_Out, 0);
		HAL_Delay(300);
		HAL_GPIO_WritePin(Buttons[3].GPIO_Out, Buttons[3].GPIO_Pin_Out, 1);
		HAL_GPIO_WritePin(Buttons[4].GPIO_Out, Buttons[4].GPIO_Pin_Out, 1);
		HAL_GPIO_WritePin(Buttons[5].GPIO_Out, Buttons[5].GPIO_Pin_Out, 1);
		HAL_GPIO_WritePin(Buttons[6].GPIO_Out, Buttons[6].GPIO_Pin_Out, 1);
}

void HoldMotor(struct Button *Button, uint8_t mode)
{

	uint32_t Limitation = 100000;
	uint16_t temp = 0;
	for(uint32_t j = 0; j < 10; j++)
	{
		HAL_GPIO_WritePin(Button->GPIO_Out, Button->GPIO_Pin_Out, 0);
		Delay(Button->Delay/2);
		HAL_GPIO_WritePin(Button->GPIO_Out, Button->GPIO_Pin_Out, 1);
		Delay(Button->Delay/2);
	}
	while(Button->B_State == mode && Limitation)
	{
		Limitation--;
		if(temp < ACCEL) temp++;
		HAL_GPIO_WritePin(Button->GPIO_Out, Button->GPIO_Pin_Out, 0);
		Delay(Button->Delay/2 - temp);
		HAL_GPIO_WritePin(Button->GPIO_Out, Button->GPIO_Pin_Out, 1);
		Delay(Button->Delay/2 - temp);
	}

}

void HoldPrepareMotorUntill(struct Button *Button, uint8_t mode)
{
	if(Button->B_State == mode)
	{
		if(flag == false)
		{
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			flag = true;
		}
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		flag = false;
		Buttons[1].B_Out = 0;
	}
}

void TimerMotor(struct Button *Button)
{
	//HAL_TIM_Base_Stop_IT(&htim7);
	uint16_t temp = 0;
	uint32_t steps = 200;
	if(Button->third_function)
	{
		Button->third_function(&steps);
	}
	for(uint32_t j = 0; j < 20; j++)
	{
		HAL_GPIO_WritePin(Button->GPIO_Out, Button->GPIO_Pin_Out, 0);
		Delay(Button->Delay/2);
		HAL_GPIO_WritePin(Button->GPIO_Out, Button->GPIO_Pin_Out, 1);
		Delay(Button->Delay/2);
	}
	for(uint32_t j = 0; j < steps; j++)
	{
		if(temp < ACCEL) temp++;
		HAL_GPIO_WritePin(Button->GPIO_Out, Button->GPIO_Pin_Out, 0);
		Delay(Button->Delay/2 - temp);
		HAL_GPIO_WritePin(Button->GPIO_Out, Button->GPIO_Pin_Out, 1);
		Delay(Button->Delay/2 - temp);
	}
	//HAL_TIM_Base_Start_IT(&htim7);
}

void SetSteps1(uint32_t* steps)
{
	*(steps) = Pullsteps;
}

void SetSteps2(uint32_t* steps)
{
	*(steps) = Dosesteps;
}

void SetSteps0(uint32_t* steps)
{
	*(steps) = 1000;
}

void Delay(uint32_t delay)
{
	while(delay--){asm("NOP");}
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
	HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, 0);
	HAL_GPIO_WritePin(Led_2_GPIO_Port, Led_2_Pin, 0);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
