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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dht11.h"
#include "fatfs_sd.h"
#include "iis3dwb_reg.h"
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
FATFS fs;
FIL fil;
uint32_t totalSpace, freeSpace;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define    BOOT_TIME        10 //ms
#define CS_up_GPIO_Port GPIOA
#define CS_up_Pin GPIO_PIN_4
#define SENSOR_BUS hspi1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t RHI, RHD, TCI, TCD, SUM;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;
uint8_t data_rec[6];
int16_t x,y,z;
float xg, yg, zg;
char buffer[100] = {0};
char data[100] = {0};
RTC_DateTypeDef getDate = {0};
RTC_TimeTypeDef getTime = {0};

static int16_t data_raw_acceleration[3];
static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float temperature_degC;
static uint8_t tx_buffer[1000];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

static int32_t platform_write (void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read (void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay (uint32_t ms);
static void platform_init (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void putData (const char* fileName, char* data) {
	f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	f_lseek(&fil, fil.fsize);
	f_puts(data, &fil);
	f_close(&fil);
}

void getDHT11 () {
	while (!DHT11_Start());

	RHI = DHT11_Read(); // Relative humidity integral
	RHD = DHT11_Read(); // Relative humidity decimal
	TCI = DHT11_Read(); // Celsius integral
	TCD = DHT11_Read(); // Celsius decimal
	SUM = DHT11_Read(); // Check sum

	if (RHI + RHD + TCI + TCD == SUM)
	{
		// Can use RHI and TCI for any purposes if whole number only needed
		tCelsius = (float)TCI + (float)(TCD/10.0);
		tFahrenheit = tCelsius * 9/5 + 32;
		RH = (float)RHI + (float)(RHD/10.0);
		// Can use tCelsius, tFahrenheit and RH for any purpose
	}

	// Send to ESP32
	sprintf(buffer, "`%f,%f,", tCelsius, RH);
	HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 1000);

	// Write to SD-Card
	sprintf(data, "%f,%f\n", tCelsius, RH);
	putData("Data.txt", data);
}


void IIS3DWB_Init (stmdev_ctx_t* dev_ctx)
{
	/* Initialize mems driver interface */
	dev_ctx->write_reg = platform_write;
	dev_ctx->read_reg = platform_read;
	dev_ctx->mdelay = platform_delay;
	dev_ctx->handle = &SENSOR_BUS;

	/* Init test platform */
	platform_init();

	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	/* Restore default configuration */
	iis3dwb_reset_set(dev_ctx, PROPERTY_ENABLE);

	/* Enable Block Data Update */
	iis3dwb_block_data_update_set(dev_ctx, PROPERTY_ENABLE);

	/* Set Output Data Rate */
	iis3dwb_xl_data_rate_set(dev_ctx, IIS3DWB_XL_ODR_26k7Hz);

	/* Set full scale */
	iis3dwb_xl_full_scale_set(dev_ctx, IIS3DWB_2g);

	/* Configure filtering chain(No aux interface)
	* Accelerometer low pass filter path
	*/
	iis3dwb_xl_filt_path_on_out_set(dev_ctx, IIS3DWB_LP_ODR_DIV_100);
}


void read_temp_iis3dwb (stmdev_ctx_t* dev_ctx) {
	uint8_t reg_temp = 0;

	while (!reg_temp) {
		iis3dwb_temp_flag_data_ready_get(dev_ctx, &reg_temp);

		if (reg_temp) {
			/* Read temperature data */
			memset(&data_raw_temperature, 0x00, sizeof(int16_t));
			iis3dwb_temperature_raw_get(dev_ctx, &data_raw_temperature);
			temperature_degC = iis3dwb_from_lsb_to_celsius(data_raw_temperature);

			// Write to SD-Card
			sprintf((char *)tx_buffer,"\nT=%4.2f\n", temperature_degC);
			putData("Data.txt", (char *) tx_buffer);
		}
	}
}

void read_acc_iis3dwb (stmdev_ctx_t* dev_ctx, uint32_t startTime) {
	uint8_t reg = 0;

	while (!reg) {
		/* Read output only if new xl value is available */
		iis3dwb_xl_flag_data_ready_get(dev_ctx, &reg);

		if (reg) {
			/* Read acceleration field data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			iis3dwb_acceleration_raw_get(dev_ctx, data_raw_acceleration);
			acceleration_mg[0] = iis3dwb_from_fs2g_to_mg(data_raw_acceleration[0]);
			acceleration_mg[1] = iis3dwb_from_fs2g_to_mg(data_raw_acceleration[1]);
			acceleration_mg[2] = iis3dwb_from_fs2g_to_mg(data_raw_acceleration[2]);

			// Write to SD-Card
			sprintf((char *)tx_buffer,"t=%f,A=%4.2f,%4.2f,%4.2f\n", (float)(HAL_GetTick() - startTime)/1000, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
			putData("Data.txt", (char *) tx_buffer);
		}
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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  	HAL_TIM_Base_Start(&htim1);
	f_mount(&fs, "", 0);

	// Switch on all the components
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);

	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);

	stmdev_ctx_t dev_ctx;
	IIS3DWB_Init(&dev_ctx);

	read_temp_iis3dwb(&dev_ctx);
	putData("Data.txt", "Acceleration data begins:\n");
	uint32_t startTime = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		read_acc_iis3dwb(&dev_ctx, startTime);
		// Send to ESP32
		sprintf(buffer, "^%f,%4.2f,%4.2f,%4.2f,", (float)(HAL_GetTick() - startTime)/1000, acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 1000);

		// Send to ESP32
		sprintf(buffer, "*%4.2f,", temperature_degC);
		HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), 1000);


		// Enter STANDBY Mode after 60 seconds
		if (HAL_GetTick() - startTime >= 60000)
		{
			// Power off all the components
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);

			// Suspend SysTick, otherwise SysTick interrupt will wakeup MCU after 1ms of STANDBY
			HAL_SuspendTick();

			// Disable Wakeup Flag, otherwise MCU will go to STANDBY only once
			(PWR->CR) |= (PWR_FLAG_WU) << 2U;

			HAL_PWR_EnterSTANDBYMode();											// Enter STANDBY Mode

			// Resume SysTick
			HAL_ResumeTick();

			// Re-init system clock configuration after STANDBY mode exit
			SystemClock_Config();

			// Switch on all the components
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);

			read_temp_iis3dwb(&dev_ctx);
			putData("Data.txt", "Acceleration data begins:\n");
			startTime = HAL_GetTick();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x1;
  sAlarm.AlarmTime.Seconds = 0x30;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA8 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Alarm goes off every 20 seconds as of now => Thus we get 20s to operate & 20s of STANDBY mode
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	RTC_AlarmTypeDef sAlarm;
	HAL_RTC_GetAlarm(hrtc, &sAlarm, RTC_ALARM_A, FORMAT_BIN);
	if (sAlarm.AlarmTime.Seconds > 58) {
		sAlarm.AlarmTime.Seconds = 0;
	}
	else {
		sAlarm.AlarmTime.Seconds = sAlarm.AlarmTime.Seconds+1;
	}
	while (HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN) != HAL_OK) {}

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	// Power on all the components
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
}


int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;

  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}


/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, 1000);
	HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
	return 0;
}


/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    reg |= 0x80;
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(handle, &reg, 1, 1000);
	HAL_SPI_Receive(handle, bufp, len, 1000);
	HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
	return 0;
}


/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}


/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
#if defined(STEVAL_MKI109V3)
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_Delay(1000);
#endif
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
