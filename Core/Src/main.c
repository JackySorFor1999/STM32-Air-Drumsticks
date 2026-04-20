/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BUF_SIZE 4096  // Total size (Ping + Pong)
#define HALF_BUF_SIZE (AUDIO_BUF_SIZE / 2)

#define HIT_THRESHOLD 30000

#define COOLDOWN_MS 120
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim6;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
#define MAX_CHANNELS 4
typedef struct {
    FIL file;
    uint8_t active;
    uint8_t tempBuf[4096];
    UINT bytesRead;
} AudioChannel;

AudioChannel channels[MAX_CHANNELS];

uint16_t audioBuffer[AUDIO_BUF_SIZE];
volatile uint8_t transferStatus = 0;

int PA0_previous = 0, PC13_previous = 0;
int PA0_current, PC13_current;

uint32_t last_hit_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_DAC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void PlayDrum(char *filename) {
    for (int i = 0; i < MAX_CHANNELS; i++) {
        if (!channels[i].active) {
            if (f_open(&channels[i].file, filename, FA_READ) == FR_OK) {
                f_lseek(&channels[i].file, 44); // Skip header
                channels[i].active = 1;

                // If DAC isn't running yet, start it
                if (HAL_DAC_GetState(&hdac) != HAL_DAC_STATE_BUSY) {
                    HAL_TIM_Base_Start(&htim6);
                    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)audioBuffer, AUDIO_BUF_SIZE, DAC_ALIGN_12B_R);
                }
            }
            return; // Found a slot, exit function
        }
    }
}

/* --- DMA Callback Functions --- */

// Called when first half of audioBuffer is played
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    transferStatus = 1;
}

// Called when second half of audioBuffer is played
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac) {
    transferStatus = 2;
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char str[12];

// The 7-bit address 0x68 shifted left for HAL becomes 0xD0
#define MPU6050_ADDR 0xD0
#define PWR_MGMT_1   0x6B

void MPU6050_Init(void) {
    uint8_t check;
    uint8_t data;

    // 1. Check if device is present (Who Am I?)
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x75, 1, &check, 1, 100);

    if (check == 0x68) { // 0x68 is the default "Who Am I" value
        // 2. Wake up the MPU6050 (Write 0 to PWR_MGMT_1)
        data = 0;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
    }
	/*if (HAL_I2C_IsDeviceReady(&hi2c1, 0xD0, 5, 100) == HAL_OK) {
	    // The chip is definitely responding at address 0xD0
	    HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x75, 1, &check, 1, 100);
	} else {
	    // The chip isn't even acknowledging the address
	}*/
}

uint8_t Rec_Data[14];
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;

void MPU6050_Read_Accel(void) {
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, 100);

    // Combine two 8-bit registers into one 16-bit value
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
}

void refresh_LCD() {
	LCD_DrawString(80, 0, "X : ");
	LCD_DrawString(80, 20, "Y : ");
	LCD_DrawString(80, 40, "Z : ");

	sprintf(str, "%5ld", Accel_X_RAW/2048);
	LCD_DrawString(120, 0, str);
	sprintf(str, "%5ld", Accel_Y_RAW/2048);
	LCD_DrawString(120, 20, str);
	sprintf(str, "%5ld", Accel_Z_RAW/2048);
	LCD_DrawString(120, 40, str);
}

/*void detect_Hit() {
	int value = Accel_Z_RAW/2048;
	// TODO: Problem: Hit up and down also detected, i.e. both are negative value.
	if (value < -9) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
}*/

bool DetectDrumHit(int16_t accel_axis) {
    uint32_t current_time = HAL_GetTick();

    // Check if we are still in the cooldown period to prevent double-triggering
    if ((current_time - last_hit_time) < COOLDOWN_MS) {
        return false;
    }

    // Check for the deceleration spike
    if (accel_axis > HIT_THRESHOLD) {
        last_hit_time = current_time; // Reset the timer
        return true;
    }

    return false;
}

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
  MX_FSMC_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_DAC_Init();
  MX_SDIO_SD_Init();
  MX_TIM6_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT();
  LCD_Clear(0, 0, 240, 320, BLACK);

  MPU6050_Init();
  MPU6050_Read_Accel();
  HAL_Delay(500);

      if (f_mount(&SDFatFS, (TCHAR const*)SDPath, 0) == FR_OK) {

    	  PlayDrum("Tom.wav"); //For testing
      }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MPU6050_Init();
	  MPU6050_Read_Accel();
	  refresh_LCD();
//	  detect_Hit();
	  if (DetectDrumHit(Accel_Z_RAW)) {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	  }
	  PA0_current = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	  PC13_current = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

	  if (PA0_previous == 0 && PA0_current == 1) PlayDrum("Tom.wav");
	  if (PC13_previous == 0 && PC13_current == 1) PlayDrum("Overhead.wav");

	  PA0_previous = PA0_current;
	  PC13_previous = PC13_current;

	  	      // 2. Audio Mixing
	  if (transferStatus > 0) {
	  	          uint32_t offset = (transferStatus == 1) ? 0 : HALF_BUF_SIZE;
	  	          transferStatus = 0;

	  	          // Create a temporary signed 32-bit mixing buffer
	  	          int32_t mixedSamples[HALF_BUF_SIZE] = {0};
	  	          uint8_t anyChannelActive = 0;

	  	          for (int ch = 0; ch < MAX_CHANNELS; ch++) {
	  	              if (channels[ch].active) {
	  	                  anyChannelActive = 1;
	  	                  if (f_read(&channels[ch].file, channels[ch].tempBuf, 4096, &channels[ch].bytesRead) != FR_OK || channels[ch].bytesRead == 0) {
	  	                      channels[ch].active = 0;
	  	                      f_close(&channels[ch].file);
	  	                  } else {
	  	                      // Add this channel's samples to the mix
	  	                      for (int i = 0; i < HALF_BUF_SIZE; i++) {
	  	                          int16_t sample = (int16_t)((channels[ch].tempBuf[2*i+1] << 8) | channels[ch].tempBuf[2*i]);
	  	                          mixedSamples[i] += sample;
	  	                      }
	  	                  }
	  	              }
	  	          }

	  	          // Convert the final mix to 12-bit Unsigned for the DAC
	  	          for (int i = 0; i < HALF_BUF_SIZE; i++) {
	  	              // Hard Clipping (Prevent distortion if too loud)
	  	              if (mixedSamples[i] > 32767) mixedSamples[i] = 32767;
	  	              if (mixedSamples[i] < -32768) mixedSamples[i] = -32768;

	  	              audioBuffer[offset + i] = (uint16_t)((mixedSamples[i] + 32768) >> 4);
	  	          }

	  	          // Optimization: If no channels are active, stop the DAC to save power/noise
	  	          if (!anyChannelActive) {
	  	              HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
	  	              HAL_TIM_Base_Stop(&htim6);
	  	              HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048); // Stay at mid-point
	  	          }
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

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 20;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1632;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /** Disconnect NADV
  */

  __HAL_AFIO_FSMCNADV_DISCONNECTED();

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
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
