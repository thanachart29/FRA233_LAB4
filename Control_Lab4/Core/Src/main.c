/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define _USE_MATH_DEFINES
#include "math.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float RadRel = 0;
void Drivemotor(float PWM);
float PID_Controller_Innerloop(float Error1);
float PID_Controller_Outerloop(float Error2);

float KalmanFilter(float Input);
uint32_t runtime = 0 ;
uint32_t storetime[2] = {0};
uint32_t deltatime = 0;
uint32_t timestamp = 0;

static enum {Init, Delay, Accerelation, Constant, Decelelation, Steadystate}state = Init;
uint32_t traject_time = 0;
float theta = 0.1;
float t_acc = 1.0;
float t_constant = 2.0;
float path = 0;
float v_profile = 0;
float t = 0;
float endtime = 0;
float v_max = 0;
float a = 0;
float voltage = 0;
float Error_theta = 0 ;
float Error_velocity = 0 ;
float Position[2] = {0};
float Velocity = 0;
float Velocity_Kalman = 0;
float velo_desire = 0 ;
float diff_theta_kalman = 0;


// Kalman Filter //
typedef struct
{
	float data[3][3];
	int rows;
	int cols;
}Matrix;

Matrix A, C, R, Q, x_k0, x_k1, z_k, xp_k, pp_k, p_k0, p_k1, A_T, C_T, y_k, s_k, s_k_inv, K, I, test;

Matrix Create_Matrix(int rows, int cols, float b[9])
{
	Matrix a;
	a.rows = rows;
	a.cols = cols;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			a.data[i][j] = 0;
		}
	}

	int n = 0;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			a.data[i][j] = 0;
			a.data[i][j] = b[n];
			n++;
		}
	}
	return a;
}

Matrix Multiply(Matrix a, Matrix b)
{
	Matrix c;
	c.rows = a.rows;
	c.cols = b.cols;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			c.data[i][j] = 0;
		}
	}

	for (int i = 0; i < a.rows; i++)
	{
		for (int j = 0; j < b.cols; j++)
		{
			for (int k = 0; k < b.rows; k++)
			{
				c.data[i][j] += a.data[i][k] * b.data[k][j];
			}
		}
	}
	return c;

}

Matrix Sum(Matrix a, Matrix b)
{
	Matrix c;
	c.rows = a.rows;
	c.cols = b.cols;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			c.data[i][j] = 0;
		}
	}

	for (int i = 0; i < a.rows; i++)
	{
		for (int j = 0; j < a.cols; j++)
		{
			c.data[i][j] = a.data[i][j] + b.data[i][j];
		}
	}

	return c;
}

Matrix Minus(Matrix a, Matrix b)
{
	Matrix c;
	c.rows = a.rows;
	c.cols = b.cols;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			c.data[i][j] = 0;
		}
	}

	for (int i = 0; i < a.rows; i++)
	{
		for (int j = 0; j < a.cols; j++)
		{
			c.data[i][j] = a.data[i][j] - b.data[i][j];
		}
	}

	return c;
}

Matrix Transpose(Matrix a)
{
	Matrix c;
	c.rows = a.rows;
	c.cols = a.cols;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			c.data[i][j] = 0;
		}
	}

	for (int i = 0; i < a.rows; i++)
	{
		for (int j = 0; j < a.cols; j++)
		{
			c.data[i][j] = a.data[j][i];
		}
	}
	return c;
}

Matrix Inverse(Matrix a)
{
	//this function is for only 1x1 matrix
	Matrix c;
	c.rows = a.rows;
	c.cols = a.cols;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			c.data[i][j] = 0;
		}
	}

	for (int i = 0; i < a.rows; i++)
	{
		for (int j = 0; j < a.cols; j++)
		{
			c.data[i][j] = 1/(a.data[i][j]);
		}
	}
	return c;
}

Matrix Store(Matrix a)
{
	Matrix c;
	c.rows = a.rows;
	c.cols = a.cols;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			c.data[i][j] = 0;
		}
	}

	for (int i = 0; i < a.rows; i++)
	{
		for (int j = 0; j < a.cols; j++)
		{
			c.data[i][j] = (a.data[i][j]);
		}
	}
	return c;
}
float sensor[1] = { 0 };
float tk = 0.00;
float ti = 0.00;
float output_theta = 0.00 ;
float zoutput_omega = 0.00 ;
int aa = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim5);

  endtime = (2.0*t_acc) + t_constant;
  v_max = 2.0*theta/(t_constant + endtime);
  a = v_max/t_acc;


  float dt = 1/100.0;
  float mat_a[9] = { 1, dt, (0.5 * (pow(dt,2))), 0, 1, dt, 0, 0, 1 };
  float mat_c[3] = { 1, 0, 0 };
  float mat_r[1] = { 0.000961807 };
  float eye[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  float zero[9] = { 0 };
  float var_jerk = 10;
  float var_theta = (pow(((1 / 6.0) * pow(dt,3)), 2))*var_jerk;
  float var_omega = (pow(((1 / 2.0) * pow(dt, 2)), 2)) * var_jerk;
  float var_alpha = (pow(dt, 2)) * var_jerk;
  float var_theta_omega = ((1 / 12.0) * pow(dt, 5)) * var_jerk;
  float var_theta_alpha = ((1 / 6.0) * pow(dt, 4)) * var_jerk;
  float var_omega_alpha = ((1 / 2.0) * pow(dt, 3))* var_jerk;
  float mat_q[9] = {var_theta, var_theta_omega, var_theta_alpha, var_theta_omega, var_omega, var_omega_alpha, var_theta_alpha, var_omega_alpha, var_alpha };

  A = Create_Matrix(3, 3, mat_a);
  A_T = Transpose(A);
  C = Create_Matrix(1, 3, mat_c);
  C_T = Transpose(C);
  R = Create_Matrix(1, 1, mat_r);
  Q = Create_Matrix(3, 3, mat_q);
  I = Create_Matrix(3, 3, eye);
  x_k1 = Create_Matrix(3, 1, zero);
  p_k1 = Create_Matrix(3, 3, zero);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  t = (HAL_GetTick() - traject_time - 1000.0)/1000.0;
	  if(t >= endtime)
	  {
		  state = Steadystate;
	  }
	  else if(t >= (t_constant + t_acc))
	  {
		  state = Decelelation;
	  }
	  else if(t >= t_acc)
	  {
		  state = Constant;
	  }
	  else if(t >= 0)
	  {
		  state = Accerelation;
	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4999;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Drivemotor(float PWM)
{
	if(PWM<0 && PWM>=-5000)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 , -1*PWM);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	}
	else if (PWM<-5000)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 , 5000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	}
	else if(PWM>=0 && PWM<=5000)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 , PWM);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	}
	else if(PWM>5000)
	{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 , 5000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	}
}

float PID_Controller_Innerloop(float Error1)
{
	static float Output1[2] = {0};
	static float error1[3] = {0};
	float Sampletime = 1/100.0 ;
	float Kp1 = 1.9899 ;
	float Ki1 = Sampletime * (7.596*1.9899);
	float Kd1 = 0.0 / Sampletime;

	error1[0] = Error1;
	Output1[0] = Output1[1] + ( (Kp1+Ki1+Kd1)*error1[0] ) - ( (Kp1 + (2*Kd1))*error1[1] ) + (Kd1*error1[2]);
	Output1[1] = Output1[0];
	error1[2] = error1[1];
	error1[1] = error1[0];

	return Output1[0];
}

float PID_Controller_Outerloop(float Error1)
{
	static float Output2[2] = {0};
	static float error2[3] = {0};
	float Sampletime = 1/100.0 ;
	float Kp2 = 18.26*1.5851 ;
	float Ki2 = Sampletime * 0.0;
	float Kd2 = 1.5851/Sampletime;

	error2[0] = Error1;
	Output2[0] = Output2[1] + ( (Kp2+Ki2+Kd2)*error2[0] ) - ( (Kp2 + (2*Kd2))*error2[1] ) + (Kd2*error2[2]);
	Output2[1] = Output2[0];
	error2[2] = error2[1];
	error2[1] = error2[0];

	return Output2[0];
}

float KalmanFilter(float Input)
{
	ti = aa / 100.0;
	tk = ti;
	sensor[0] = Input;
	z_k = Create_Matrix(1, 1, sensor);
	//Predict//
	xp_k = Multiply(A, x_k1);
	pp_k = Sum(Multiply(Multiply(A, p_k1), A_T), Q);
	//Update//
	y_k = Minus(z_k, Multiply(C, xp_k));
	s_k = Sum(Multiply(Multiply(C, pp_k), C_T), R);
	s_k_inv = Inverse(s_k);
	K = Multiply(Multiply(pp_k, C_T), s_k_inv);
	x_k0 = Sum(xp_k, Multiply(K, y_k));
	p_k0 = Multiply(Minus(I, Multiply(K, C)), pp_k);

	diff_theta_kalman = (x_k0.data[0][0] - x_k1.data[0][0])/(10.0/1000.0);

	//Memory [N-1] data
	x_k1 = Store(x_k0);
	p_k1 = Store(p_k0);
	output_theta = x_k0.data[0][0];
	zoutput_omega = x_k0.data[1][0];
	return zoutput_omega;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim5)
	{
		timestamp = HAL_GetTick();
		storetime[0] = HAL_GetTick();
		RadRel = (TIM2->CNT/3999.0)*(2*M_PI);
		Position[0] = RadRel;
		Velocity = (Position[0] - Position[1])/(10.0/1000.0);
		switch (state)
		{
		case Init:
			traject_time = timestamp;
			state = Delay;
			break;
		case Delay:
			path = 0;
			v_profile = 0;
			break;
		case Accerelation:
			path = 0.5*a*(pow(t,2));
			v_profile = a*(t);
			break;
		case Constant:
			path = (v_max*(t-t_acc)) + (0.5*a*(pow(t_acc,2)));
			v_profile = v_max;
			break;
		case Decelelation:
			path = (v_max*(t-(t_constant+t_acc))) - (0.5*a*(pow(t-(t_constant+t_acc),2))) + (v_max*t_constant) + (0.5*a*(pow(t_acc,2)));
			v_profile = v_max-(a*(t-(t_constant+t_acc)));
			break;
		case Steadystate:
			path = theta;
			v_profile = 0;
			break;
		default:
			break;

		}

		Error_theta = path - RadRel;
		velo_desire = PID_Controller_Outerloop(Error_theta);
		Velocity_Kalman = KalmanFilter(RadRel);
		//Error_velocity = velo_desire - Velocity;
		Error_velocity = v_profile + velo_desire - Velocity_Kalman;
		voltage = PID_Controller_Innerloop(Error_velocity);



		Drivemotor((voltage/12)*5000.0);




		deltatime = storetime[0] - storetime[1];
		storetime[1] = storetime[0];
		Position[1] = Position[0];
		runtime = HAL_GetTick() - timestamp;
	}
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

