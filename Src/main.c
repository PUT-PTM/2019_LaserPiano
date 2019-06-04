/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "MY_CS43L22.h"
#include <math.h>
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

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define PI 3.14159f
#define F_SAMPLE			48000.0f
//#define F_OUT				1760.0f
#define OCTAVE				36
float mySinVal;
float sample_dt;
uint16_t sample_N[OCTAVE];
int16_t dataI2S[OCTAVE][100];
int reset[8];	// when equals 0 turns of sound
int onesecond = 300; // 100/100Hz
int oldNotes[2];
int resetNotes[2];
#define C6 (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3))
#define D6 (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4))
#define E6 (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5))
#define F6 (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6))
#define G6 (HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8))
#define A6 (HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9))
#define H6 (HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10))
#define C7 (HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11))

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void generate_samples(int fout[OCTAVE])	// generates sinus waves
{
	for(int k =0; k<OCTAVE; k++)
	  {
		sample_dt = fout[k]/F_SAMPLE;
		sample_N[k] = F_SAMPLE/fout[k];
		for(uint16_t i=0; i<sample_N[k]; i++)
	  	{
	  		mySinVal = sinf(i*2*PI*sample_dt);
	  		dataI2S[k][i*2] = (mySinVal )*8000;    //Right data (0 2 4 6 8 10 12)
	  		dataI2S[k][i*2 + 1] =(mySinVal )*8000; //Left data  (1 3 5 7 9 11 13)
	  	}
	  }
}

void calculateNotes(int toReturn[28], int fout[OCTAVE])	// estimate combination of notes
{
	int x=0;
	for(int j =0; j<8; j++) //calculates 28 combinations of notes
	{
			for(int k=j+1; k<8; k++)
			{
				toReturn[x++]=abs(fout[j]-fout[k]);
			}
	}
}

int calculateNoteCobination(int note1, int note2) // returns index of combined notes
{
	if(note1>note2)	// swaps notes
	{
		note1 = note1^note2;
		note2 = note2^note1;
		note1 = note1^note2;
	}
	if(note2==0)
	{
		return note1 - 1;
	}
	else if(note2==1)
	{
		return note1 + 5;
	}
	else if(note2==2)
	{
		return note1 + 10;
	}
	else if(note2==3)
	{
		return note1 + 14;
	}
	else if(note2==4)
	{
		return note1 + 17;
	}
	else if(note2==5)
	{
		return note1 + 19;
	}
	else if(note2==6)
	{
		return note1 + 20;
	}
}

void switchNotes(int note1, int note2)	// switch current notes
{
	HAL_I2S_DMAStop(&hi2s3);
	if(note2 < 0 && note1>=0)	// if only one note is playing
	{
		HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dataI2S[note1], sample_N[note1]*2);
	}
	else if(note1>=0 && note2>=0)	// if two notes is playing
	{
	  	HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dataI2S[calculateNoteCobination(note1, note2)],
	  			sample_N[calculateNoteCobination(note1, note2)]*2);
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{

	if(htim->Instance == TIM4)
	{
		for(int i = 0; i < 8; i++)
		{
			if(reset[i]>=0)	// counts down time, when note should stop playing
			{
				reset[i]--;
			}
		}
		for(int i = 0; i < 8; i++)
		{
				if(reset[i] < 0) // when note should stop playing
				{
					if(resetNotes[0] == i)
					{
						if(resetNotes[1]>=0)
						{
							resetNotes[0]=resetNotes[1];
							resetNotes[1]=-1;
						}
						else
						{
							resetNotes[0]=-1;
						}
					}
					else if(resetNotes[1] == i)
					{
						resetNotes[1]=-1;
					}
				}
			}
		if ((oldNotes[0]!=resetNotes[0]) || (oldNotes[1]!=resetNotes[1])) // switches notes
		{
			oldNotes[0] = resetNotes[0];
			oldNotes[1] = resetNotes[1];
			switchNotes(resetNotes[0], resetNotes[1]);
		}
	}
}

void setReset(int i)	// sets reset note after one second
{
	if(resetNotes[0] == -1)
	{
		resetNotes[0] = i;
		reset[i] = onesecond;	// turns of sound after one second
	}
	else if(resetNotes[1] == -1)
	{
		resetNotes[1] = i;
		reset[i] = onesecond;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//Turns the sound on
{

	if(C6 == GPIO_PIN_RESET)
	{
		//Note C6
		setReset(0);
	}
	if(D6 == GPIO_PIN_RESET)
	{
		//Note D6
		setReset(1);
	}
	if(E6 == GPIO_PIN_RESET)
	{
		//Note E6
		setReset(2);
	}
	if(F6 == GPIO_PIN_RESET)
	{
		//Note F6
		setReset(3);
	}
	if(G6 == GPIO_PIN_RESET)
	{
		//Note G6
		setReset(4);
	}
	if(A6 == GPIO_PIN_RESET)
	{
		//Note A6
		setReset(5);
	}
	if(H6 == GPIO_PIN_RESET)
	{
		//Note H6
		setReset(6);
	}
	if(C7 == GPIO_PIN_RESET)
	{
		//Note C7
		setReset(7);
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int notes[8] = {1047,1175,1319,1397,1568,1760,1976,2093}; // notes frequency
  for(int i=0;i<8;i++) 
  {
	  reset[i] = 0;
  }
  oldNotes[0] = oldNotes[1] = -1; // turns off notes on start
  resetNotes[0] = resetNotes[1] = -1;
  int calculatedNotes[28];
  calculateNotes(calculatedNotes, notes); // sets up all combined notes
  int fout[OCTAVE];
  for(int i=0;i<8;i++){
	  fout[i]=notes[i];
  }
  int j=0;
  for(int i=8;i<OCTAVE;i++){
  	  fout[i]=calculatedNotes[j];
  	  j++;
    }

  generate_samples(fout);

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  CS43_Init(hi2c1, MODE_I2S);
  CS43_SetVolume(80);
  CS43_Enable_RightLeft(CS43_RIGHT_LEFT);
  CS43_Start();
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 167;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 PE4 PE5 PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
