/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define Sen2_Pin GPIO_PIN_2
#define Sen2_GPIO_Port GPIOE
#define M1InB_Pin GPIO_PIN_3
#define M1InB_GPIO_Port GPIOE
#define Sen3_Pin GPIO_PIN_4
#define Sen3_GPIO_Port GPIOE
#define M2InA_Pin GPIO_PIN_5
#define M2InA_GPIO_Port GPIOE
#define Sen4_Pin GPIO_PIN_6
#define Sen4_GPIO_Port GPIOE
#define M2InB_Pin GPIO_PIN_13
#define M2InB_GPIO_Port GPIOC
#define Sen5_Pin GPIO_PIN_14
#define Sen5_GPIO_Port GPIOC
#define M3InA_Pin GPIO_PIN_15
#define M3InA_GPIO_Port GPIOC
#define Sen6_Pin GPIO_PIN_0
#define Sen6_GPIO_Port GPIOC
#define M3InB_Pin GPIO_PIN_1
#define M3InB_GPIO_Port GPIOC
#define Sen7_Pin GPIO_PIN_2
#define Sen7_GPIO_Port GPIOC
#define photoRes_Pin GPIO_PIN_3
#define photoRes_GPIO_Port GPIOC
#define TX_Pin GPIO_PIN_0
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_1
#define RX_GPIO_Port GPIOA
#define Sen8_Pin GPIO_PIN_4
#define Sen8_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_6
#define SERVO_GPIO_Port GPIOA
#define M2Pwm_Pin GPIO_PIN_1
#define M2Pwm_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOE
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define BnoReset_Pin GPIO_PIN_11
#define BnoReset_GPIO_Port GPIOD
#define M3Pwm_Pin GPIO_PIN_12
#define M3Pwm_GPIO_Port GPIOD
#define OP8_Pin GPIO_PIN_13
#define OP8_GPIO_Port GPIOD
#define M4Pwm_Pin GPIO_PIN_14
#define M4Pwm_GPIO_Port GPIOD
#define OP7_Pin GPIO_PIN_15
#define OP7_GPIO_Port GPIOD
#define M6Pwm_Pin GPIO_PIN_6
#define M6Pwm_GPIO_Port GPIOC
#define M5Pwm_Pin GPIO_PIN_7
#define M5Pwm_GPIO_Port GPIOC
#define OP6_Pin GPIO_PIN_8
#define OP6_GPIO_Port GPIOC
#define OP5_Pin GPIO_PIN_9
#define OP5_GPIO_Port GPIOC
#define Brush1_Pin GPIO_PIN_8
#define Brush1_GPIO_Port GPIOA
#define OP4_Pin GPIO_PIN_9
#define OP4_GPIO_Port GPIOA
#define Brush2_Pin GPIO_PIN_10
#define Brush2_GPIO_Port GPIOA
#define OP3_Pin GPIO_PIN_11
#define OP3_GPIO_Port GPIOA
#define OP2_Pin GPIO_PIN_15
#define OP2_GPIO_Port GPIOA
#define OP1_Pin GPIO_PIN_11
#define OP1_GPIO_Port GPIOC
#define En4_In_Pin GPIO_PIN_12
#define En4_In_GPIO_Port GPIOC
#define En3_In_Pin GPIO_PIN_1
#define En3_In_GPIO_Port GPIOD
#define M4InA_Pin GPIO_PIN_2
#define M4InA_GPIO_Port GPIOD
#define En2_In_Pin GPIO_PIN_3
#define En2_In_GPIO_Port GPIOD
#define M4InB_Pin GPIO_PIN_4
#define M4InB_GPIO_Port GPIOD
#define En4_Ext_Pin GPIO_PIN_5
#define En4_Ext_GPIO_Port GPIOD
#define En4_Ext_EXTI_IRQn EXTI9_5_IRQn
#define M5InA_Pin GPIO_PIN_6
#define M5InA_GPIO_Port GPIOD
#define En3_Ext_Pin GPIO_PIN_7
#define En3_Ext_GPIO_Port GPIOD
#define En3_Ext_EXTI_IRQn EXTI9_5_IRQn
#define M5InB_Pin GPIO_PIN_3
#define M5InB_GPIO_Port GPIOB
#define En1_In_Pin GPIO_PIN_4
#define En1_In_GPIO_Port GPIOB
#define M6InA_Pin GPIO_PIN_5
#define M6InA_GPIO_Port GPIOB
#define En2_Ext_Pin GPIO_PIN_6
#define En2_Ext_GPIO_Port GPIOB
#define En2_Ext_EXTI_IRQn EXTI9_5_IRQn
#define M6InB_Pin GPIO_PIN_7
#define M6InB_GPIO_Port GPIOB
#define En1_Ext_Pin GPIO_PIN_8
#define En1_Ext_GPIO_Port GPIOB
#define En1_Ext_EXTI_IRQn EXTI9_5_IRQn
#define M1Pwm_Pin GPIO_PIN_9
#define M1Pwm_GPIO_Port GPIOB
#define Sen1_Pin GPIO_PIN_0
#define Sen1_GPIO_Port GPIOE
#define M1InA_Pin GPIO_PIN_1
#define M1InA_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define buzzer HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET)
#define silent HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET)

#define val1  HAL_GPIO_ReadPin(Sen1_GPIO_Port, Sen1_Pin)
#define val2  HAL_GPIO_ReadPin(Sen2_GPIO_Port, Sen2_Pin)
#define val3  HAL_GPIO_ReadPin(Sen3_GPIO_Port, Sen3_Pin)
#define val4  HAL_GPIO_ReadPin(Sen4_GPIO_Port, Sen4_Pin)
#define val5  HAL_GPIO_ReadPin(Sen5_GPIO_Port, Sen5_Pin)
#define val6  HAL_GPIO_ReadPin(Sen6_GPIO_Port, Sen6_Pin)
#define val7  HAL_GPIO_ReadPin(Sen7_GPIO_Port, Sen7_Pin)
#define val8  HAL_GPIO_ReadPin(Sen8_GPIO_Port, Sen8_Pin)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
