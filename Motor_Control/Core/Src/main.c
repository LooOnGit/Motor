/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "tm1637.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUTOMATIC 1
#define MANUAL 0
#define RELEASE 1
#define PRESS 0
#define NORMAL 1
#define LOAD_HIGH 0

#define PWM_FULL_DUTY 50
#define MOTOR_MAX_SPEED 280 //280rpmm
#define PULSES_REVOLUTION 234//Pulses per channel per revolution

#define TIME_SLOW_STARTUP 4000 //4s
#define TIME_SLOW_START 10000 //10s

#define D 3//diameter (cm)

#define VOL_SUPPLY_MCU 3300 //ms
#define ADC_12_BIT 4095
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int load;
typedef struct {
	bool step1;
	bool step2;
	bool step3;
	bool foot;
	bool side;
	bool retract;
	bool autoManual;
}	Button;

typedef struct {
	float poten1;
	float poten2;
	float pwmValue;
	int setRate;
	int rate;
	bool loadStatus;
}	Motor;

typedef struct {
	int timeSysStart;
	int timeSysPre;
	int timePressFootStart;
	int timePressFootPre;
} Time;

Button button;
Motor motor;
Time time;

int retractCm = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void slowStartup();
void startup();
void setSpeed();
void displaySpeed(int n);
int adjustSpeedDisplay();
void manualAutomatic();
void slowStart(int n);
void retract();
void resetStatusButton();
int measureSpeed();
bool checkLoadHigh();

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TM1637_CLK_PORT GPIOA
#define TM1637_CLK_PIN GPIO_PIN_5
#define TM1637_DIO_PORT GPIOA
#define TM1637_DIO_PIN GPIO_PIN_7
TM1637Display display;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  tm1637Init();
  tm1637SetBrightness(3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);

  button.autoManual = MANUAL;
  button.step1 = RELEASE;
  button.step2 = RELEASE;
  button.step3 = RELEASE;
  button.foot = RELEASE;
  button.side = RELEASE;
  button.retract = RELEASE;

  motor.loadStatus =NORMAL;

  time.timeSysStart = HAL_GetTick();
  TM1637_Init(&display, TM1637_CLK_PORT, TM1637_CLK_PIN, TM1637_DIO_PORT, TM1637_DIO_PIN, 5);
  TM1637_SetBrightness(&display, 7, 1);
  TM1637_ShowNumberDec(&display, 1234, 0, 4, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  retractCm = 0;
	 if(button.side == PRESS){
		 slowStart(75);
		 resetStatusButton();
	 }

	 if(button.autoManual == AUTOMATIC){
		 if(button.foot == PRESS){
			 if((HAL_GetTick() - time.timePressFootStart) >= 2000){
				 resetStatusButton();
				 slowStartup();
				 // Run with speed config				 while(1){
					 setSpeed();
					 motor.rate = measureSpeed();
					 displaySpeed(motor.rate);
					 if(motor.rate |= motor.setRate){
						 break;
					 }
				 }
				 adjustSpeedDisplay();

				 //check load high
				 if(!checkLoadHigh()){
					while(1){
						if(button.foot == PRESS){
							if((HAL_GetTick() - time.timePressFootStart) >= 2000){
								break;
							}
						 }
						if(button.foot == RELEASE){
							continue;
						}
					}
				 }
			 }
		 }
	 else{
		 if(button.foot == PRESS){
			 resetStatusButton();
			 slowStartup();
			 // Run with speed config
			 while(1){
				 setSpeed();
				 motor.rate = measureSpeed();
				 displaySpeed(motor.rate);
				 if(motor.rate |= motor.setRate){
					 break;
				 }
			 }
			 adjustSpeedDisplay();

			 //check load high
			 if(!checkLoadHigh()){
				if(button.foot == RELEASE){
					continue;
				}
			 }
		 }
	 }


	 if(((HAL_GetTick() - time.timeSysStart) > 5000) && (button.retract == PRESS)){
		  retract();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 49;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TM1637_CLK_Pin|TM1637_DIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TM1637_CLK_Pin TM1637_DIO_Pin */
  GPIO_InitStruct.Pin = TM1637_CLK_Pin|TM1637_DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M_A_SW_Pin Retract_SW_Pin Rotary_SW_1_Pin Rotary_SW_2_Pin
                           Rotary_SW_3_Pin Foot_SW_Pin BTN_Side_Pin */
  GPIO_InitStruct.Pin = M_A_SW_Pin|Retract_SW_Pin|Rotary_SW_1_Pin|Rotary_SW_2_Pin
                          |Rotary_SW_3_Pin|Foot_SW_Pin|BTN_Side_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : nSLEEP_Pin */
  GPIO_InitStruct.Pin = nSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nSLEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nFAULT_Pin */
  GPIO_InitStruct.Pin = nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nFAULT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void slowStartup(){
	float step = PWM_FULL_DUTY/(TIME_SLOW_STARTUP/10.0);
	motor.pwmValue = 0;
	while(motor.pwmValue < PWM_FULL_DUTY) {
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor.pwmValue);
	    motor.pwmValue += step;
	    HAL_Delay(10);
	}
}


void setSpeed(){
	HAL_ADC_Start(&hadc1);
	HAL_Delay(1000);
	int adc1 = HAL_ADC_GetValue(&hadc1)*VOL_SUPPLY_MCU/ADC_12_BIT;
	HAL_ADC_Stop(&hadc1);
	int poten1 = (adc1*PWM_FULL_DUTY)/VOL_SUPPLY_MCU;

	// display rate to poten
	tm1637SetBrightness(8);
	motor.poten1 = (poten1*MOTOR_MAX_SPEED)/PWM_FULL_DUTY;
	tm1637DisplayDecimal(motor.poten1, 0);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, poten1);
}

int measureSpeed(){
	int timeStart = HAL_GetTick();
	int index = 0;
	int encoderCnt = 0;
	int encoderCntPre = 0;
	int rate = 0;
	while(index < 3){
		if((HAL_GetTick() - timeStart) >= 1000){
			encoderCnt = __HAL_TIM_GET_COUNTER(&htim1);
			rate = (encoderCnt - encoderCntPre)/PULSES_REVOLUTION;
			encoderCntPre = encoderCnt;
			timeStart = HAL_GetTick();
			index++;
		}
	}
	motor.rate = rate*60;
	return rate;
}

void displaySpeed(int n){
	int N = 0.079746*(n*60);
	TM1637_ShowNumberDec(&display, N, 0, 4, 0);
}

int adjustSpeedDisplay(){
	int poten;
	if(button.step1 == PRESS){
		poten = PWM_FULL_DUTY;
		motor.setRate = (poten*MOTOR_MAX_SPEED)/PWM_FULL_DUTY;
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, poten);
		displaySpeed(poten);
	}
	if(button.step2 == PRESS){
		poten = PWM_FULL_DUTY/3;
		motor.setRate = (poten*MOTOR_MAX_SPEED)/PWM_FULL_DUTY;
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, poten);
		displaySpeed(poten);
	}
	if(button.step3 == PRESS){
		poten = PWM_FULL_DUTY/5;
		motor.setRate = (poten*MOTOR_MAX_SPEED)/PWM_FULL_DUTY;
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, poten);
		displaySpeed(poten);
	}
	return poten;
}

void slowStart(int n){
	int setPWM = (n*PWM_FULL_DUTY)/MOTOR_MAX_SPEED; //0-50
	float step = setPWM/(TIME_SLOW_START/10.0);

	motor.pwmValue = 0;
	while(motor.pwmValue < setPWM) {
	    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, motor.pwmValue);
	    motor.pwmValue += step;
	    HAL_Delay(10);
	}
}

void retract(){
	HAL_ADC_Start(&hadc2);
	HAL_Delay(1000);
	int adc2 = HAL_ADC_GetValue(&hadc2)*VOL_SUPPLY_MCU/ADC_12_BIT;
	HAL_ADC_Stop(&hadc2);
	int poten = (adc2*PWM_FULL_DUTY)/VOL_SUPPLY_MCU;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, poten);

	retractCm += (measureSpeed()*(-1))*3.14*D;
	tm1637SetBrightness(8);
	tm1637DisplayDecimal(retractCm, 0);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		button.autoManual = AUTOMATIC;
	}
	if(GPIO_Pin == GPIO_PIN_1){
		button.retract = PRESS;
	}
	if(GPIO_Pin == GPIO_PIN_2){
		button.step1 = PRESS;
		button.step2 = RELEASE;
		button.step3 = RELEASE;
	}
	if(GPIO_Pin == GPIO_PIN_10){
		button.step1 = RELEASE;
		button.step2 = PRESS;
		button.step3 = RELEASE;
	}
	if(GPIO_Pin == GPIO_PIN_11){
		button.step1 = RELEASE;
		button.step2 = RELEASE;
		button.step3 = PRESS;
	}
	if(GPIO_Pin == GPIO_PIN_13){
		button.side = PRESS;
	}
	if(GPIO_Pin == GPIO_PIN_12){
		button.foot=!button.foot;
		time.timePressFootStart = HAL_GetTick();
	}
	if(GPIO_Pin == GPIO_PIN_15){
		motor.loadStatus = LOAD_HIGH;
	}
}

bool checkLoadHigh(){
	if(motor.loadStatus == LOAD_HIGH)
		return true;
	else
		return false;
}

void resetStatusButton(){
	  button.autoManual = MANUAL;
	  button.foot = RELEASE;
	  button.side = RELEASE;
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
