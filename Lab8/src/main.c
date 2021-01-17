/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "lcd.c"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

uint8_t _displayfunction;
uint8_t _displaycontrol;
uint8_t _displaymode;
uint8_t _initialized;
uint8_t _numlines;
uint8_t _currline;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init_OC(void);
static void MX_TIM2_Init_IC(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void delayMilli(int ms);
void lcdInit(uint8_t cols, uint8_t lines, uint8_t dotsize);
void lcdPrint(char *str);
void lcdSetRGB(uint8_t r, uint8_t g, uint8_t b);
void lcdHome();
void lcdClear();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

unsigned char greenBt[] = "Verde\n";
unsigned char redBt[] = "Rojo\n";
unsigned char waitBt[] = "Esperando Verde\n";
uint8_t rxBufferBT[2];
volatile int signalTimer = 0;
volatile int captura = 0;
volatile unsigned int medicion[2] = {0,0};
volatile enum semaphore {ST_GREEN,ST_YELLOW,ST_RED,ST_BLINKGREEN,ST_STATICRED} g_next_state ;

int getDistance(){
  HAL_TIM_OC_Start_IT(&htim2,TIM_CHANNEL_1);
  MX_TIM2_Init_IC();
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
  delayMilli(50);
  MX_TIM2_Init_OC();

  int distance = ((medicion[1]-medicion[0])*10/58);
  signalTimer = 0;
  captura = 0;
  return distance;
}

void Semaforo(void){
  
  switch(g_next_state){
    case ST_GREEN:
      printf("VERDE\r\n");
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//apago amarillo
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //enciendo verde peatones
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //apago rojo conductores
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET); //enciendo verde conductores
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); //enciendo rojo peatones
      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,75); //ponemos barrera a 90º
      break;
      
    case ST_YELLOW:
      printf("Amarillo\r\n");
      g_next_state=ST_RED;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); //apago rojo conductores
      HAL_UART_Transmit(&huart6,waitBt,sizeof(waitBt),HAL_MAX_DELAY); //envio por Bluetooth "esperando verde"
      lcdSetRGB(0,0,255);
      lcdClear();
      lcdPrint("ESPERE VERDE");
      if(getDistance() <= 400){ //si hay un objeto a menos de 400 espero
        delayMilli(3000);
      }
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //apago led verde conductores
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); //enciendo amarillo conductores
      delayMilli(3000);
      break;           

    case ST_RED:
      g_next_state=ST_BLINKGREEN;
      printf("Rojo\r\n");
	    HAL_UART_Transmit(&huart6,greenBt,sizeof(greenBt),HAL_MAX_DELAY); //envio por Bluetooth "verde"
      lcdSetRGB(0,255,0);
      lcdClear();
      lcdPrint("PASE");
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//apago amarillo
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);//apago rojo peatones
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);//enciendo verde peatones
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//enciendo rojo conductores
      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,125); //ponemos barrera a 180º
      delayMilli(15000);
      break;

    case ST_STATICRED:
    printf("Rojo estatico\r\n");
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); //apago led verde conductores
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);//apago amarillo
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);//apago rojo peatones
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);//enciendo verde peatones
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);//enciendo rojo conductores
      __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,125); //ponemos barrera a 180º
      break;

    case ST_BLINKGREEN:
    printf("Verde parpadea\r\n");
    g_next_state= ST_GREEN;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //apago verde peatones
      for (int i = 0; i < 3; i++)
      {
        delayMilli(500);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //enciendo verde peatones
        delayMilli(500);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //apago verde peatones
      }
      HAL_UART_Transmit(&huart6,redBt,sizeof(redBt),HAL_MAX_DELAY); //envio por Bluetooth "rojo"
      lcdSetRGB(255,0,0);
      lcdClear();
      lcdPrint("PULSE BOTON");
      break;
  }
}

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim11);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  MX_TIM2_Init_OC();
  /* USER CODE END 2 */
  lcdInit(10,2,1);
  HAL_UART_Transmit(&huart6,redBt,sizeof(redBt),HAL_MAX_DELAY); //envio por Bluetooth "rojo"
  HAL_UART_Receive_IT(&huart6,&rxBufferBT,sizeof(rxBufferBT));
  lcdSetRGB(255,0,0);
  lcdPrint("PULSE BOTON");
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    Semaforo();
    
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void delayMilli(int ms){
  ms = ms * 2;
  __HAL_TIM_SET_COUNTER(&htim11,0);
  while(__HAL_TIM_GET_COUNTER(&htim11) < ms);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == ButtonD7_Pin ){
    g_next_state = ST_YELLOW; 
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){	

  if(rxBufferBT[0] == 10){
    switch (rxBufferBT[1]){
    case 0:
      __set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));//desactivo la interrupcion del boton
      g_next_state = ST_STATICRED;
      lcdSetRGB(0,255,0);
      lcdClear();
      lcdPrint("PASE");
      HAL_UART_Transmit(&huart6,greenBt,sizeof(greenBt),HAL_MAX_DELAY); //envio por Bluetooth "verde"
      break;

    case 1:
      __set_BASEPRI(1 << (8 - __NVIC_PRIO_BITS));//desactivo la interrupcion del boton
      //cuando esta la luz en amarillo g_next_state marca el siguiente estado desde el principio
      if(g_next_state == ST_RED || g_next_state == ST_GREEN){
        g_next_state = ST_GREEN;
      }else{
        g_next_state = ST_BLINKGREEN;
      }
      break;

    case 2:
      if(g_next_state == ST_STATICRED){
        g_next_state = ST_BLINKGREEN;
      }else{
        g_next_state=ST_GREEN;
      }
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
      //__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_8);
      NVIC_ClearPendingIRQ(EXTI9_5_IRQn);     
      __set_BASEPRI(0);
      break;
    }
  }

  HAL_UART_Receive_IT(&huart6,&rxBufferBT,sizeof(rxBufferBT));
}

void HAL_TIM_OC_DelayElapsedCallback	(	TIM_HandleTypeDef *htim	){
    signalTimer++;
    if(signalTimer == 2){
      HAL_TIM_OC_Stop_IT(&htim2,TIM_CHANNEL_1);
    }  
}

void HAL_TIM_IC_CaptureCallback	(	TIM_HandleTypeDef *htim	){
  medicion[captura] = HAL_TIM_ReadCapturedValue( &htim2, TIM_CHANNEL_1 );
  captura++;
  if(captura == 2){
    HAL_TIM_IC_Stop_IT(&htim2 , TIM_CHANNEL_1);
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_TIM2_Init_IC(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}


/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init_OC(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1679;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 124;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 41999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 0xFFFF;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  //htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ButtonD7_Pin */
  GPIO_InitStruct.Pin = ButtonD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ButtonD7_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
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


int __io_putchar(int ch)
{
  uint8_t c[1];
  c[0] = ch & 0x00FF;
  HAL_UART_Transmit(&huart2, &*c, 1, 100);
  return ch;
}

int _write(int file,char *ptr, int len)
{
  int DataIdx;
  for(DataIdx= 0; DataIdx< len; DataIdx++)
  {
    __io_putchar(*ptr++);
  }
  return len;
}


void i2c_send_byte(uint8_t dta){
    HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, &dta, sizeof(dta) , 0x100);
}

void i2c_send_byteS(uint8_t *dta, uint8_t len){
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, dta, len , 0x100);
}

void lcdCommand(uint8_t value){
    uint8_t dta[2] = {0x80, value};
    i2c_send_byteS(dta, 2);
}

void lcdSetReg(uint8_t addr, uint8_t data){
    uint8_t dta[] = {addr, data};
    HAL_I2C_Master_Transmit(&hi2c1, RGB_ADDRESS, dta, sizeof(dta) , 0x100);
}

void lcdDisplay() {
    _displaycontrol |= LCD_DISPLAYON;
    lcdCommand(LCD_DISPLAYCONTROL | _displaycontrol);
}


void lcdClear(){
    lcdCommand(LCD_CLEARDISPLAY);        // rgb_lcd_clear rgb_lcd_display, set cursor position to zero
    delayMilli(2);          // this rgb_lcd_command takes a long time!
}

void lcdHome(){
    lcdCommand(LCD_RETURNHOME);        // set cursor position to zero
    delayMilli(2);        // this rgb_lcd_command takes a long time!
}

void lcdSetRGB(uint8_t r, uint8_t g, uint8_t b){
    lcdSetReg(REG_RED, r);
    lcdSetReg(REG_GREEN, g);
    lcdSetReg(REG_BLUE, b);
}

void lcdWrite(uint8_t value){
    uint8_t dta[2] = {0x40, value};
    i2c_send_byteS(dta, 2);
}

void lcdPrint(char *str){
    while(*str) {
        lcdWrite(*str);
        str++;
    }
}

void lcdSetCursor(uint8_t col, uint8_t row){
    col = (row == 0 ? col|0x80 : col|0xc0);
    uint8_t dta[2] = {0x80, col};
    i2c_send_byteS(dta, 2);
}

void lcdInit(uint8_t cols, uint8_t lines, uint8_t dotsize){
    if (lines > 1) {
        _displayfunction |= LCD_2LINE;
    }
    _numlines = lines;
    _currline = 0;

    // for some 1 line displays you can select a 10 pixel high font
    if ((dotsize != 0) && (lines == 1)) {
        _displayfunction |= LCD_5x10DOTS;
    }

    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50

    delayMilli(50);

    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set rgb_lcd_command sequence
    lcdCommand(LCD_FUNCTIONSET | _displayfunction);
    delayMilli(5);  // wait more than 4.1ms

    // second try
    lcdCommand(LCD_FUNCTIONSET | _displayfunction);
    delayMilli(1);

    // third go
    lcdCommand(LCD_FUNCTIONSET | _displayfunction);


    // finally, set # lines, font size, etc.
    lcdCommand(LCD_FUNCTIONSET | _displayfunction);

    // turn the rgb_lcd_display on with no cursor or blinking default
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcdDisplay();

    // rgb_lcd_clear it off
    lcdClear();

    // Initialize to default text direction (for romance languages)
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    // set the entry mode
    lcdCommand(LCD_ENTRYMODESET | _displaymode);


    // backlight init
    lcdSetReg(0, 0);
    lcdSetReg(1, 0);
    lcdSetReg(0x08, 0xAA);     // all led control by pwm

    lcdSetCursor(0, 0);
    
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/