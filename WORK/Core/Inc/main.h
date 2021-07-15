/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OUT_B_Pin GPIO_PIN_2
#define OUT_B_GPIO_Port GPIOE
#define Button_2_Pin GPIO_PIN_3
#define Button_2_GPIO_Port GPIOE
#define Button_1_Pin GPIO_PIN_4
#define Button_1_GPIO_Port GPIOE
#define CS_Pin GPIO_PIN_5
#define CS_GPIO_Port GPIOE
#define CounterReset_Pin GPIO_PIN_6
#define CounterReset_GPIO_Port GPIOE
#define HOLD_Dose_Pin GPIO_PIN_13
#define HOLD_Dose_GPIO_Port GPIOC
#define Size_Select_Pin GPIO_PIN_1
#define Size_Select_GPIO_Port GPIOC
#define Led_1_Pin GPIO_PIN_6
#define Led_1_GPIO_Port GPIOA
#define Led_2_Pin GPIO_PIN_7
#define Led_2_GPIO_Port GPIOA
#define V_Push_V_Weld_Pin GPIO_PIN_7
#define V_Push_V_Weld_GPIO_Port GPIOE
#define Prepare_material_Pin GPIO_PIN_8
#define Prepare_material_GPIO_Port GPIOE
#define Dose_Pin GPIO_PIN_9
#define Dose_GPIO_Port GPIOE
#define V_Push_Pin GPIO_PIN_10
#define V_Push_GPIO_Port GPIOE
#define V_Weld_Pin GPIO_PIN_11
#define V_Weld_GPIO_Port GPIOE
#define H_Push_Pin GPIO_PIN_12
#define H_Push_GPIO_Port GPIOE
#define H_Weld_Pin GPIO_PIN_13
#define H_Weld_GPIO_Port GPIOE
#define Pull_Pin GPIO_PIN_14
#define Pull_GPIO_Port GPIOE
#define Cut_Pin GPIO_PIN_15
#define Cut_GPIO_Port GPIOE
#define Mode_Pin GPIO_PIN_10
#define Mode_GPIO_Port GPIOB
#define Auto_Start_Pin GPIO_PIN_11
#define Auto_Start_GPIO_Port GPIOB
#define Dose_Pulse_Out_Pin GPIO_PIN_8
#define Dose_Pulse_Out_GPIO_Port GPIOD
#define Dose_Out_Pin GPIO_PIN_9
#define Dose_Out_GPIO_Port GPIOD
#define V_Push_Out_Pin GPIO_PIN_10
#define V_Push_Out_GPIO_Port GPIOD
#define V_Weld_Out_Pin GPIO_PIN_11
#define V_Weld_Out_GPIO_Port GPIOD
#define H_Push_Out_Pin GPIO_PIN_12
#define H_Push_Out_GPIO_Port GPIOD
#define H_Weld_Out_Pin GPIO_PIN_13
#define H_Weld_Out_GPIO_Port GPIOD
#define Pull_Out_Pin GPIO_PIN_14
#define Pull_Out_GPIO_Port GPIOD
#define Cut_Out_Pin GPIO_PIN_15
#define Cut_Out_GPIO_Port GPIOD
#define Prepare_OUT_Pin GPIO_PIN_6
#define Prepare_OUT_GPIO_Port GPIOC
#define Reed_Switch_Pin GPIO_PIN_3
#define Reed_Switch_GPIO_Port GPIOD
#define STOP_Pin GPIO_PIN_4
#define STOP_GPIO_Port GPIOD
#define RST_Pin GPIO_PIN_7
#define RST_GPIO_Port GPIOB
#define OUT_A_Pin GPIO_PIN_0
#define OUT_A_GPIO_Port GPIOE
#define HOLD_Pull_Pin GPIO_PIN_1
#define HOLD_Pull_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
