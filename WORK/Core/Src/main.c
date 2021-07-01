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

#define STEPS 4600
#define ACCEL 800

#define PULL_STEPS 1590
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

extern char tx_buffer[128]; //Буфер для отправки текста на дисплей
extern uint8_t Frame_buffer[1024]; //Буфер кадра

static uint32_t Pullsteps = PULL_STEPS, Dosesteps = STEPS;
uint32_t counter = 0, count_prev = 0;

char tmpflg = 0;

static bool flag = false;

#define BUTTONS_COUNT 16
struct Button Buttons[BUTTONS_COUNT];
static uint32_t Defines[BUTTONS_COUNT][6] = {
		{(uint32_t)V_Push_V_Weld_GPIO_Port, V_Push_V_Weld_Pin, 0, 0, PRESS, 0},															// 0Macros
		{(uint32_t)Prepare_material_GPIO_Port, Prepare_material_Pin, (uint32_t)0, 0, HOLD_UNTIL, 12000},								// 1Prepare material from bobbin
		{(uint32_t)Dose_GPIO_Port, Dose_Pin, (uint32_t)Dose_Out_GPIO_Port, Dose_Out_Pin, TIMER, 2580},									// 2Dose motor control
		{(uint32_t)V_Push_GPIO_Port, V_Push_Pin, (uint32_t)V_Push_Out_GPIO_Port, V_Push_Out_Pin, TOGGLE, 0},							// 3Vertical cylinder push
		{(uint32_t)V_Weld_GPIO_Port, V_Weld_Pin, (uint32_t)V_Weld_Out_GPIO_Port, V_Weld_Out_Pin, __DELAY, 300},							// 4Vertical Weld spot
		{(uint32_t)H_Push_GPIO_Port, H_Push_Pin, (uint32_t)H_Push_Out_GPIO_Port, H_Push_Out_Pin, TOGGLE, 0},							// 5Horizontal cylinder push
		{(uint32_t)H_Weld_GPIO_Port, H_Weld_Pin, (uint32_t)H_Weld_Out_GPIO_Port, H_Weld_Out_Pin, __DELAY, 300},							// 6Horizontal Weld spot
		{(uint32_t)Pull_GPIO_Port, Pull_Pin, (uint32_t)Pull_Out_GPIO_Port, Pull_Out_Pin, TIMER, 2600},									// 7Pull material down
		{(uint32_t)Cut_GPIO_Port, Cut_Pin, (uint32_t)Cut_Out_GPIO_Port, Cut_Out_Pin, HOLD, 0},											// 8Cutter
		{(uint32_t)Mode_GPIO_Port, Mode_Pin, 0, 0, HOLD, 0},																			// 9Mode select
		{(uint32_t)Auto_Start_GPIO_Port, Auto_Start_Pin, 0, 0, TOGGLE, 0},																// 10Auto mode start
		{(uint32_t)HOLD_Dose_GPIO_Port, HOLD_Dose_Pin, (uint32_t)Dose_Out_GPIO_Port, Dose_Out_Pin, HOLD_MOTOR, 2580},					// 11HOLD_Dose motor
		{(uint32_t)HOLD_Pull_GPIO_Port, HOLD_Pull_Pin, (uint32_t)Pull_Out_GPIO_Port, Pull_Out_Pin, HOLD_MOTOR, 2580},					// 12HOLD_Pull motor
		{(uint32_t)Reed_Switch_GPIO_Port, Reed_Switch_Pin, (uint32_t)0, 0, 200, 2580},													// 13Reed Switch feedback
		{(uint32_t)CounterReset_GPIO_Port, CounterReset_Pin, (uint32_t)0, 0, 0, 0},														// 14Counter reset
		{(uint32_t)Size_Select_GPIO_Port, Size_Select_Pin, (uint32_t)0, 0, 0, 0}														// 15Pocket size select
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
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
  MX_DMA_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  GPIOD->ODR = ~0;
  Init();
  HAL_Delay(700);
  ST7920_Clean();
  sprintf(tx_buffer, "Count: %lu", counter);
  ST7920_Decode_UTF8(20, 4, 0, tx_buffer);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{

		if(!Buttons[9].B_State)
		{
			tmpflg = 1;
			if(Buttons[9].B_Out == 1)
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
			}
			//AUTO MODE
			#define PUSH_V HAL_GPIO_WritePin(Buttons[3].GPIO_Out, Buttons[3].GPIO_Pin_Out, 0); // Push
			#define PUSH_H HAL_GPIO_WritePin(Buttons[5].GPIO_Out, Buttons[5].GPIO_Pin_Out, 0); // Push
			#define RELEASE_H HAL_GPIO_WritePin(Buttons[3].GPIO_Out, Buttons[3].GPIO_Pin_Out, 1); //Release
			#define RELEASE_V HAL_GPIO_WritePin(Buttons[5].GPIO_Out, Buttons[5].GPIO_Pin_Out, 1); //Release
			#define CUT_START HAL_GPIO_WritePin(Buttons[8].GPIO_Out, Buttons[8].GPIO_Pin_Out, 0); // CUT
			#define CUT_RELEASE HAL_GPIO_WritePin(Buttons[8].GPIO_Out, Buttons[8].GPIO_Pin_Out, 1); //Release
			#define WELD_H_START HAL_GPIO_WritePin(Buttons[4].GPIO_Out, Buttons[4].GPIO_Pin_Out, 0); // Weld
			#define WELD_V_START HAL_GPIO_WritePin(Buttons[6].GPIO_Out, Buttons[6].GPIO_Pin_Out, 0); // Weld
			#define WELD_H_STOP HAL_GPIO_WritePin(Buttons[4].GPIO_Out, Buttons[4].GPIO_Pin_Out, 1);
			#define WELD_V_STOP HAL_GPIO_WritePin(Buttons[6].GPIO_Out, Buttons[6].GPIO_Pin_Out, 1);
			#define PULL TimerMotor(&Buttons[7]); // PULL mat.
			#define DOSE TimerMotor(&Buttons[2]); // Dose
			#define CYCLE_DELAY HAL_Delay(75);
			#define WELD_TIME HAL_Delay(300);
			if(Buttons[10].B_Out) // AUTO MODE START
			{

				if(Buttons[15].B_State == 1)
				{
					Pullsteps = PULL_STEPS / 2;
					Dosesteps = STEPS;
					//Pull new material
					PULL

					CYCLE_DELAY
					//Clamp material
					PUSH_H
					PUSH_V

					CYCLE_DELAY
					//Welding stage 1
					WELD_H_START
					WELD_V_START
					WELD_TIME
					WELD_H_STOP
					WELD_V_STOP
					RELEASE_H
					RELEASE_V

					CYCLE_DELAY
					//Pull stage 2
					PULL

					CYCLE_DELAY
					//Clamp material
					PUSH_V

					CYCLE_DELAY
					//Welding stage 2
					WELD_V_START
					WELD_TIME
					WELD_V_STOP
					CUT_START
					CYCLE_DELAY
					//Cut, fill, release
					CUT_RELEASE
					DOSE
					RELEASE_V

					CYCLE_DELAY
				}
				else
				{
					Pullsteps = PULL_STEPS / 2;
					Dosesteps = STEPS / 2;
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
				}
				HAL_Delay(200);
				counter++;
			}
			else if(flag == true)
			{
				flag = false;
				HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			}
			sprintf(tx_buffer, "Count: %lu", counter);
			ST7920_Decode_UTF8(20, 4, 0, tx_buffer);
			ST7920_Update();
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 24000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led_1_Pin|Led_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Dose_Out_Pin|V_Push_Out_Pin|V_Weld_Out_Pin|H_Push_Out_Pin 
                          |H_Weld_Out_Pin|Pull_Out_Pin|Cut_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : HOLD_Dose_Pin Size_Select_Pin */
  GPIO_InitStruct.Pin = HOLD_Dose_Pin|Size_Select_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_1_Pin Led_2_Pin */
  GPIO_InitStruct.Pin = Led_1_Pin|Led_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Mode_Pin Auto_Start_Pin */
  GPIO_InitStruct.Pin = Mode_Pin|Auto_Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Dose_Out_Pin V_Push_Out_Pin V_Weld_Out_Pin H_Push_Out_Pin 
                           H_Weld_Out_Pin Pull_Out_Pin Cut_Out_Pin */
  GPIO_InitStruct.Pin = Dose_Out_Pin|V_Push_Out_Pin|V_Weld_Out_Pin|H_Push_Out_Pin 
                          |H_Weld_Out_Pin|Pull_Out_Pin|Cut_Out_Pin;
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
	  ST7920_Init();
	  ST7920_Graphic_mode(1);
	  sprintf(tx_buffer, "WELCOME");
	  ST7920_Decode_UTF8(50, 3, 0, tx_buffer);
	  ST7920_Update();
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
			HAL_TIM_Base_Stop_IT(&htim7);
			Delay(4800000);
			while(HAL_GPIO_ReadPin(STOP_GPIO_Port, STOP_Pin) == 0){asm("NOP");};
			HAL_NVIC_SystemReset();
		}
		for(uint8_t i = 0; i < BUTTONS_COUNT; i++)
		{
			if((Buttons[i].addiction->B_Out == 1 && Buttons[i].Mode == __DELAY) || Buttons[i].addiction == 0 || Buttons[i].Mode != __DELAY)
				Buttons[i].call_function(&Buttons[i]);
		}

		if(Buttons[14].B_Out && counter > 0)
		{
			Buttons[14].B_Out = 0;
			counter = 0;
			ST7920_Clean();
		}
		if(Buttons[10].B_Out)
		{
			HoldPrepareMotorUntill(Buttons[1].addiction, 1);
		}
	}
	if (htim->Instance==TIM6)
	{
		ST7920_Update();
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

	uint16_t Limitation = 10000;
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
