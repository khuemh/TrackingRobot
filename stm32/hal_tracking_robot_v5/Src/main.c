/*
**
**                           Main.c
**
**
*******************************************************************************/
/*
   Last committed:     $Revision: 00
   Last changed by:    $Author: KhueHM
   Last changed date:  $Date:  $
   ID:                 $Id:  $

*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "delay.h" 
#include "pid.h"

const int speed = 1100;

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
/* Hardware Initialization */
void hardware_init(void);
/* LEFT motor control functions */
void leftMotorClkWise(uint16_t speed);
void leftMotorCounterClkWise(uint16_t speed);
void leftMotorStop(void);
/* RIGHT motor control functions */
void rightMotorClkWise(uint16_t speed);
void rightMotorCounterClkWise(uint16_t speed);
void rightMotorStop(void);
/* ROBOT movement control */
void robotForward(uint16_t speed);
void robotBack(uint16_t speed);
void robotTurnLeft(uint16_t speed);
void robotTurnRight(uint16_t speed);
void robotStop(void);
/* Camera servo control */
void cam_tilt(uint16_t angle);
void cam_tilt_UP(void);
void cam_tilt_DOWN(void);
void cam_toggle(uint8_t n);
/* SRF05 functions */
uint8_t Read_Distance(void);
/* Direct LED */
void led_DIR(uint8_t LED);
void led_DIR_toggle(uint8_t n);
void led_DIR_circle(uint8_t n, uint8_t delay);

/* Global variables ----------------------------------------------------------*/                                    
volatile uint8_t receive_data[rx_Size];
uint16_t obj_X;
uint16_t obj_Y;
uint16_t obj_A;

const uint16_t center_X = 320;
const uint16_t center_Y = 240;
const uint8_t center_W = 100;
const uint16_t obj_A_ST = 2000;

const uint8_t servoStep = 2;
uint16_t servo_duty = 1600;
const uint8_t cam_tilt_MAX = 130;
const uint8_t cam_tilt_MIN = 0;
const int8_t servo_calib = 0;
uint16_t servoPos;

uint8_t distance;

bool streaming_mode = 0;
bool tracking_mode = 0;

/* MAIN FUNCTIONS ------------------------------------------------------------*/
int main(void)
{
	hardware_init();
  
  for(uint8_t i = 0; i < rx_Size; i++)
  {
    receive_data[i] = 0;
  }

  robotStop();
  cam_toggle(3);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1600);

  led_DIR_toggle(4);

  while (1)
  {	
		distance = Read_Distance();

    obj_X = receive_data[0] << 8 | receive_data[1];
	  obj_Y = receive_data[2] << 8 | receive_data[3];
	  obj_A = receive_data[4] << 8 | receive_data[5];		
		
		
		if (streaming_mode)
		{
				if (distance <= 10) 
				{
					robotStop();
					led_DIR(0);
				}
				else
				{
					/* Robot Control */
					if ((obj_X != 0) && (obj_Y != 0)) 
					{
					/* YYY Control */
					if ((obj_Y > 0) && (obj_Y < (center_Y - (center_W)))) 
					{
						servo_duty -= 10;
					}
					else if ((obj_Y > center_Y + (center_W)) && (obj_Y < center_Y * 2))
					{
						servo_duty += 10;
					}
					// else
					// {
					//   continue;
					// }
					if (servo_duty > 2000) 
					{
						servo_duty = 2000;
					}
					else if (servo_duty < 630)
					{
						servo_duty = 630;
					}
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo_duty);
					 /* YYY Control */
						
						/* XXX Control */
						if ((obj_X > 0) && (obj_X < (center_X - center_W)))
						{
							robotTurnRight(speed);
							led_DIR(1);
						}
						else if ((obj_X > (center_X + center_W)) && (obj_X < center_X * 2))
						{
							robotTurnLeft(speed);
							led_DIR(3);
						} /* XXX Control */
						else
						{ /* AAA Control */
							if (obj_A > 12000) 
							{
								robotBack(2000);
								led_DIR(2);
							}
							else if (obj_A < 5000)    
							{
								robotForward(2000);
								led_DIR(4);
							}
							else
							{
								robotStop();
								led_DIR(0);
							} /* AAA Control */
						} 
					} /* Robot Control */      
				}
    
				HAL_Delay(50);
		}
		else
		{
			led_DIR_circle(4, 50);
			robotStop();
		}

	} /* While */

}

/* User Interrupt functions -------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance)
	{

	}
}

void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	streaming_mode = HAL_GPIO_ReadPin(stream_mode_GPIO_Port, stream_mode_Pin);
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI6_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	tracking_mode = HAL_GPIO_ReadPin(track_mode_GPIO_Port, track_mode_Pin);
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}

/* USER'S FUNCTIONS ----------------------------------------------------------*/
/* Hardware initialization */
void hardware_init(void)
{
	/* System's initialization */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
	
	/* User's initialization */
  DWT_Delay_Init();
	
	HAL_SPI_Receive_DMA(&hspi3, (uint8_t*)receive_data, rx_Size);
	
  HAL_TIM_MspPostInit(&htim1);
	HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	
  HAL_TIM_MspPostInit(&htim2);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
  HAL_TIM_Base_Start_IT(&htim3);
}

/* Direct LED */
void led_DIR(uint8_t LED)
{
  switch (LED)
  {
    case 1:
      set(led_R_GPIO_Port, led_R_Pin);
      reset(led_G_GPIO_Port, led_G_Pin);
      reset(led_O_GPIO_Port, led_O_Pin);
      reset(led_B_GPIO_Port, led_B_Pin);
      break;
    case 2:
      reset(led_R_GPIO_Port, led_R_Pin);
      reset(led_G_GPIO_Port, led_G_Pin);
      reset(led_O_GPIO_Port, led_O_Pin);
      set(led_B_GPIO_Port, led_B_Pin);
      break;
    case 3:
      reset(led_R_GPIO_Port, led_R_Pin);
      set(led_G_GPIO_Port, led_G_Pin);
      reset(led_O_GPIO_Port, led_O_Pin);
      reset(led_B_GPIO_Port, led_B_Pin);
      break;
    case 4:
      reset(led_R_GPIO_Port, led_R_Pin);
      reset(led_G_GPIO_Port, led_G_Pin);
      set(led_O_GPIO_Port, led_O_Pin);
      reset(led_B_GPIO_Port, led_B_Pin);
      break;
    case 5:
      set(led_R_GPIO_Port, led_R_Pin);
      set(led_G_GPIO_Port, led_G_Pin);
      set(led_O_GPIO_Port, led_O_Pin);
      set(led_B_GPIO_Port, led_B_Pin);
      break;
  
    default:
      reset(led_R_GPIO_Port, led_R_Pin);
      reset(led_G_GPIO_Port, led_G_Pin);
      reset(led_O_GPIO_Port, led_O_Pin);
      reset(led_B_GPIO_Port, led_B_Pin);
      break;
  }
}

void led_DIR_toggle(uint8_t n)
{
  for(uint8_t i = 0; i < n; i++)
  {
    led_DIR(5);
    HAL_Delay(300);
    led_DIR(0);
    HAL_Delay(300);
  }
}

void led_DIR_circle(uint8_t n, uint8_t delay)
{
  for (uint8_t i = 0; i < n; i++)
  {
    set(led_R_GPIO_Port, led_R_Pin);
    HAL_Delay(delay);
    reset(led_R_GPIO_Port, led_R_Pin);
    HAL_Delay(delay);
    set(led_B_GPIO_Port, led_B_Pin);
    HAL_Delay(delay);
    reset(led_B_GPIO_Port, led_B_Pin);
    HAL_Delay(delay);
    set(led_G_GPIO_Port, led_G_Pin);
    HAL_Delay(delay);
    reset(led_G_GPIO_Port, led_G_Pin);
    HAL_Delay(delay);
    set(led_O_GPIO_Port, led_O_Pin);
    HAL_Delay(delay);
    reset(led_O_GPIO_Port, led_O_Pin);
    HAL_Delay(delay);
  }
  
}

/* LEFT motor control functions */
void leftMotorClkWise(uint16_t speed)
{
    set(left_IN1_GPIO_Port, left_IN1_Pin);
    reset(left_IN2_GPIO_Port, left_IN2_Pin);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

void leftMotorCounterClkWise(uint16_t speed)
{
    reset(left_IN1_GPIO_Port, left_IN1_Pin);
    set(left_IN2_GPIO_Port, left_IN2_Pin);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
}

void leftMotorStop(void)
{
    reset(left_IN1_GPIO_Port, left_IN1_Pin);
    reset(left_IN2_GPIO_Port, left_IN2_Pin);
}

/* RIGHT motor control functions */
void rightMotorClkWise(uint16_t speed)
{
    set(right_IN1_GPIO_Port, right_IN1_Pin);
    reset(right_IN2_GPIO_Port, right_IN2_Pin);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}

void rightMotorCounterClkWise(uint16_t speed)
{
    reset(right_IN1_GPIO_Port, right_IN1_Pin);
    set(right_IN2_GPIO_Port, right_IN2_Pin);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);
}

void rightMotorStop(void)
{
    reset(right_IN1_GPIO_Port, right_IN1_Pin);
    reset(right_IN2_GPIO_Port, right_IN2_Pin);
}

/* ROBOT movement control */
void robotForward(uint16_t speed)
{
    leftMotorClkWise(speed);
    rightMotorClkWise(speed);
}
void robotBack(uint16_t speed)
{
    leftMotorCounterClkWise(speed);
    rightMotorCounterClkWise(speed);
}
void robotTurnLeft(uint16_t speed)
{
    leftMotorCounterClkWise(speed);
    rightMotorClkWise(speed);
}
void robotTurnRight(uint16_t speed)
{
    leftMotorClkWise(speed);
    rightMotorCounterClkWise(speed);
}
void robotStop(void)
{
    leftMotorStop();
    rightMotorStop();
}

/* Camera Servo control functions */
void cam_tilt(uint16_t angle)
{
  angle += servo_calib;
	if (angle > cam_tilt_MAX + servo_calib) 
  {
    angle = cam_tilt_MAX + servo_calib;
  }
  else if (angle < cam_tilt_MIN + servo_calib)
  {
    angle = cam_tilt_MIN + servo_calib;
  }
  
  servoPos = angle - servo_calib;
  uint16_t tmp = angle*10 + 600;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tmp);
}

void cam_tilt_UP(void)
{
    cam_tilt(servoPos);
    HAL_Delay(50);
    servoPos -= 1;
}

void cam_tilt_DOWN(void)
{
    cam_tilt(servoPos);
    HAL_Delay(50);
    servoPos += 1;
}

void cam_toggle(uint8_t n)
{
  for(uint8_t i = 0; i < n; i++)
  {
    cam_tilt(30);
    HAL_Delay(500);
    cam_tilt(120);
    HAL_Delay(500);
  }
}

/* SRF05 */
uint8_t Read_Distance(void)
{
	__IO bool flag = 0;
	__IO uint32_t disTime = 0;
	
  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
	DWT_Delay_us(5);
  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

	while(flag == 0)
	{ 
    while(HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) == SET)
		{
	  	disTime++;
	    flag = 1;
		}	
	}
	return (disTime / 294.12);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = tim1_PRSCL;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = tim1_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = tim3_PRSCL;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = tim3_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, left_IN1_Pin|left_IN2_Pin|right_IN1_Pin|right_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, trig_Pin|led_G_Pin|led_O_Pin|led_R_Pin 
                          |led_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : stream_mode_Pin track_mode_Pin */
  GPIO_InitStruct.Pin = stream_mode_Pin|track_mode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : left_IN1_Pin left_IN2_Pin right_IN1_Pin right_IN2_Pin */
  GPIO_InitStruct.Pin = left_IN1_Pin|left_IN2_Pin|right_IN1_Pin|right_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : trig_Pin */
  GPIO_InitStruct.Pin = trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(trig_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : echo_Pin */
  GPIO_InitStruct.Pin = echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(echo_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : led_G_Pin led_O_Pin led_R_Pin led_B_Pin */
  GPIO_InitStruct.Pin = led_G_Pin|led_O_Pin|led_R_Pin|led_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
