/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include "ADC.h"
#include "Delay.h"
#include "LowPass_FirstOrder.h"
#include "HighPass_FirstOrder.h"
#include "PeakingFilter.h"
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

#define BUFFER_SIZE 128
#define INT16_TO_FLOAT (1.0f/32768.0f)
#define FLOAT_TO_INT16 32768.0f
#define SAMPLING_RATE 	48000
#define SIN_FREQ		1000
#define BUFFER_LENGTH 	SAMPLING_RATE / SIN_FREQ
#define VOLUME 1.5

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac2;

I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s1;
I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

/* USER CODE BEGIN PV */

int16_t inData[BUFFER_SIZE] __attribute__((aligned(4)));

static int16_t i2s_buffer_mic1[BUFFER_SIZE];  // Mic1 - I2S3
static int16_t i2s_buffer_mic2[BUFFER_SIZE];  // Mic2 - I2S2


static volatile int16_t *inBufPtr;


uint16_t dac_value;
uint16_t dataReadyFlag = 0;

float outTest;


//HighPass_FirstOrder hpFilt;
//LowPass_FirstOrder lpFilt;
EQ eq;
Delay dly;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	inBufPtr = &inData[0];
	dataReadyFlag = 1;
}

void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	inBufPtr = &inData[BUFFER_SIZE/2];
	dataReadyFlag = 1;
}

void processDataNoDelay() {

	float left;

	for (uint8_t n = 0; n < (BUFFER_SIZE/2) - 1; n += 1) {
		left = (inBufPtr[n] * INT16_TO_FLOAT);// + 0.0537;

		if (left > 1.0f) {
			left = 1.0f;
		}
		else if (left < -1.0f) {
			left = -1.0f;
		}

		//left = LowPass_FirstOrder_Update(&lpFilt, left);
		//left = HighPass_FirstOrder_Update(&hpFilt, left);

		dac_value = (int16_t)(left * 2047.0f * VOLUME) + 2048;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);


	}

}



void processData(LowPass_FirstOrder *lpFilt, HighPass_FirstOrder *hpFilt) {


	static float left;
	static float right;
	static float rightDelayed;
	static float out;

	//float out = Delay_Update(&dly, sample_i2s) * 5;
	for (uint8_t n = 0; n < (BUFFER_SIZE/2) - 1; n += 2) {

		left = (inBufPtr[n] * INT16_TO_FLOAT); // + 0.0537;


		//left = LowPass_FirstOrder_Update(lpFilt, left);
		//left = HighPass_FirstOrder_Update(hpFilt, left);

		if (left > 1.0f) {
			left = 1.0f;
		}
		else if (left < -1.0f) {
			left = -1;

		}

		right = (inBufPtr[n+1] * INT16_TO_FLOAT); // + 0.054565;


		//right = LowPass_FirstOrder_Update(lpFilt, right);
		//right = HighPass_FirstOrder_Update(hpFilt, right);
		rightDelayed = Delay_Update(&dly, right);

		if (rightDelayed > 1.0f) {
			rightDelayed = 1.0f;
		}
		else if (rightDelayed < -1.0f) {
			rightDelayed = -1;
		}

		out = left - rightDelayed;



		out = LowPass_FirstOrder_Update(lpFilt, out);
		out = EQ_filter(&eq, out);
		out = out * VOLUME;

		if (out > 1.0f) {
			out = 1.0f;
		}
		else if (out < -1.0f) {
			out = -1;
		}

		outTest = out;

		dac_value = (uint16_t)((out + 1.0f) * 2047.5f);

		if (dac_value > 4095) {
			out = 4095;
		}

		outTest = out;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);
		//DACPtr[n] = dac_value;
		//DACPtr[n+1] = dac_value;
		//printf("n: %d \n", n);
		//printf("left: %d \n", (int16_t)(left*32768.0f));
		//printf("right: %d \n", (int16_t)(rightDelayed*32768.0f));


	}

}

/*
void processDataADC() {
	float in;

	for (int n = 0; n < BUFFER_LENGTH*2; n++) {
		in = sin(2 * 3.14 * SIN_FREQ * n / SAMPLING_RATE);



		dac_value = (int16_t)(in*VOLUME);

		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_value);
	}

}

*/

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

  //__HAL_RCC_SYSCLK_CONFIG(RCC_CFGR_SWS_HSI);
  // HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  //HAL_NVIC_EnableIRQ(SysTick_IRQn);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_I2C3_Init();
  MX_I2S3_Init();
  MX_I2S2_Init();
  MX_I2S1_Init();
  /* USER CODE BEGIN 2 */

/*
 *
 */



  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_I2S_Receive_DMA(&hi2s3, (uint16_t *)inData, BUFFER_SIZE);
  HAL_I2S_Receive_DMA(&hi2s1 , (uint16_t *)i2s_buffer_mic1, BUFFER_SIZE); // Mic1
  HAL_I2S_Receive_DMA(&hi2s2, (uint16_t *)i2s_buffer_mic2, BUFFER_SIZE); // Mic2
  //HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint16_t*)DacBuffer, BUFFER_SIZE, DAC_ALIGN_12B_R);
  // 500 hz: delay = 1 ms
  float delayTime = 0.4081632653;;
  Delay_Init(&dly, delayTime, 1, 0, SAMPLING_RATE);



  //ADC_Init();

  LowPass_FirstOrder *lpFilt = (LowPass_FirstOrder*)malloc(sizeof(LowPass_FirstOrder));
  HighPass_FirstOrder *hpFilt = (HighPass_FirstOrder*)malloc(sizeof(HighPass_FirstOrder));

  if (!lpFilt || !hpFilt) {
      Error_Handler();  // Or handle it gracefully
  }

  LowPass_FirstOrder_Init(lpFilt, 2000.0f, SAMPLING_RATE);
  HighPass_FirstOrder_Init(hpFilt, 100.0f, SAMPLING_RATE);
  EQ_init(&eq, SAMPLING_RATE);
  EQ_setParam(&eq, 150, 1.5, 50);
  //EQ_setParam()

  //HAL_ADC_Start(&hadc1);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	   if (dataReadyFlag) {
		   dataReadyFlag = 0;
		   processData(lpFilt, hpFilt);
	    //processDataNoDelay();
	    //processDataADC();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1|RCC_PERIPHCLK_I2S_APB2;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 13;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

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
  hi2s3.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
int _write(int file, char *ptr, int len) {
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
*/

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
