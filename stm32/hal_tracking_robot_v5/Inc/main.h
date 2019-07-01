/*
**
**                           Main.h
**
**
*******************************************************************************/
/*
   Last committed:     $Revision: 00
   Last changed by:    $Author: KhueHM
   Last changed date:  $Date:  $
   ID:                 $Id:  $

*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define tim1_PRSCL								200
#define tim1_PERIOD								13929

#define tim3_PRSCL                64
#define tim3_PERIOD               25845

#define stream_mode_Pin 					GPIO_PIN_2
#define stream_mode_GPIO_Port 		GPIOA
#define stream_mode_EXTI_IRQn 		EXTI2_IRQn

#define track_mode_Pin 						GPIO_PIN_6
#define track_mode_GPIO_Port 			GPIOA
#define track_mode_EXTI_IRQn 			EXTI9_5_IRQn

#define SERVO_Pin 								GPIO_PIN_0
#define SERVO_GPIO_Port 					GPIOA

#define left_IN1_Pin 							GPIO_PIN_7
#define left_IN1_GPIO_Port 				GPIOE
#define left_IN2_Pin 							GPIO_PIN_8
#define left_IN2_GPIO_Port 				GPIOE
#define left_EN_Pin 							GPIO_PIN_9
#define left_EN_GPIO_Port 				GPIOE

#define right_IN1_Pin 						GPIO_PIN_10
#define right_IN1_GPIO_Port 			GPIOE
#define right_IN2_Pin 						GPIO_PIN_12
#define right_IN2_GPIO_Port 			GPIOE
#define right_EN_Pin 							GPIO_PIN_11
#define right_EN_GPIO_Port 				GPIOE

#define SPI_NSS_Pin 							GPIO_PIN_4
#define SPI_NSS_GPIO_Port 				GPIOA
#define SPI_SCK_Pin 							GPIO_PIN_12
#define SPI_SCK_GPIO_Port 				GPIOB
#define SPI_MISO_Pin 							GPIO_PIN_11
#define SPI_MISO_GPIO_Port 				GPIOC
#define SPI_MOSI_Pin 							GPIO_PIN_12
#define SPI_MOSI_GPIO_Port 				GPIOC

#define trig_Pin                  GPIO_PIN_10
#define trig_GPIO_Port            GPIOD
#define echo_Pin                  GPIO_PIN_11
#define echo_GPIO_Port            GPIOD

#define led_G_Pin                 GPIO_PIN_12
#define led_G_GPIO_Port           GPIOD
#define led_O_Pin                 GPIO_PIN_13
#define led_O_GPIO_Port           GPIOD
#define led_R_Pin                 GPIO_PIN_14
#define led_R_GPIO_Port           GPIOD
#define led_B_Pin                 GPIO_PIN_15
#define led_B_GPIO_Port           GPIOD

/* DEFINE --------------------------------------------------------------------*/
#define rx_Size			              6u
#define set(GPIOx, GPIO_Pin)      HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
#define reset(GPIOx, GPIO_Pin)    HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
