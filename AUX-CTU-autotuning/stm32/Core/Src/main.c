/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STATE_IDLE, STATE_CALIBRATING, STATE_DEPLOY
} state_t;

typedef enum {
	EVENT_OFF_BUTTON,
	EVENT_CALIBRATE_DONE,
	EVENT_DEPLOY_BUTTON,
	EVENT_ALERT,
} event_t;

typedef enum {
	NONE, OV, OC, OT, HS, DI,
} alert_t;

typedef struct {
	state_t currentState;
	event_t event;
	state_t nextState;
	void (*action)(void);
} transition_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADD_TEMP1 0x48
#define I2C_ADD_TEMP2 0x49
#define I2C_ADD_PWR 0x40

#define ADC_BUF_LEN 151

#define BUFFER_SIZE 1024

static double TEMP_LIMIT = 40;
static double CURRENT_LIMIT = 2.2;
static double VOLT_LIMIT = 70;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

HRTIM_HandleTypeDef hhrtim1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
state_t currentState = STATE_IDLE;
bool running = true;
static volatile uint8_t keepCalState = 0;

static volatile uint16_t adc_buf[ADC_BUF_LEN];

static uint8_t rxBuffer[BUFFER_SIZE];
static volatile uint16_t rxIndex = 0;

static volatile uint8_t jsonFlag = 0;
static volatile bool UART_READY = true;
static volatile bool SEND_TIMER_FLAG = false;

alert_t gAlertType = NONE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void handleEvent(event_t event);
void actionTurnOff();
void actionTurnOn();
void actionCalibrate();

void parse_json(const char *json);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// @formatter:off
transition_t transitionTable[] = {
		{ STATE_IDLE, 			EVENT_DEPLOY_BUTTON,    STATE_CALIBRATING, 	actionCalibrate },

		{ STATE_CALIBRATING, 	EVENT_CALIBRATE_DONE, 	STATE_DEPLOY, 	    actionTurnOn },

		{ STATE_DEPLOY, 		EVENT_ALERT, 			STATE_IDLE,      	actionTurnOff },
		{ STATE_DEPLOY, 		EVENT_OFF_BUTTON, 		STATE_IDLE,     	actionTurnOff },
		{ STATE_CALIBRATING, 	EVENT_ALERT, 			STATE_IDLE, 		actionTurnOff },
		{ STATE_CALIBRATING, 	EVENT_OFF_BUTTON, 		STATE_IDLE, 		actionTurnOff },
};
//@formatter:on
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) adc_buf, ADC_BUF_LEN);

	// Buffers for I2C
	uint8_t aTxRegPtr[1];
	uint8_t aTxData[2];

	// Set current sensor averaging
	aTxRegPtr[0] = 0x00;
	aTxData[0] = 0b01000111;
	aTxData[1] = 0b00100111;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t) I2C_ADD_PWR << 1U,
			(uint16_t) aTxRegPtr[0], 1, (uint8_t*) aTxData, 2, HAL_MAX_DELAY);

	double initial_duty = 0.30;
	double current_duty = initial_duty;
	double cal_duty = current_duty;
	double max_duty = 0.40;
	double min_duty = 0.20;
	uint16_t signal_period = 604;
	uint16_t compare1_val = initial_duty * signal_period;
	uint16_t compare3_val = compare1_val + signal_period / 2;

	HAL_HRTIM_WaveformCountStart(&hhrtim1,
	HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_B);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,
	HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
			HRTIM_COMPAREUNIT_1, compare1_val);
	__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
			HRTIM_COMPAREUNIT_3, compare3_val);
	HAL_HRTIM_SoftwareUpdate(&hhrtim1,
	HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B | HRTIM_TIMERUPDATE_MASTER);

	int16_t tuning;
	uint16_t adc_max;
	uint8_t low_vds_count;
	bool limit_reached = false;

	uint8_t zero_cross = 103;
	uint8_t low_vds_count_threshold = current_duty * ADC_BUF_LEN - 1;
	uint16_t low_vds_threshold = 200;
	uint16_t vds_checking_threshold = 1800;
	int16_t tuning_threshold = 400;

	char json[1024];

	HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1); // Enable UART receive interrupt
	HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		adc_max = 0;
		low_vds_count = 0;
		uint16_t adc_buf2[ADC_BUF_LEN];

		/* Transfer buffer and find maximum value */
		for (int i = 0; i < ADC_BUF_LEN; i++) {
			adc_buf2[i] = adc_buf[i]; // convert to real value
			if (adc_buf2[i] > adc_max) {
				adc_max = adc_buf2[i];	// find maximum value in buffer
			}
		}

		/* Count how much values are 'zero' */
		for (int i = 0; i < ADC_BUF_LEN; i++) {
			if (adc_buf2[i] < low_vds_threshold
					&& adc_max > vds_checking_threshold) {
				low_vds_count++;
			}
		}

		/* Find hard-switching value */
		tuning = adc_buf2[zero_cross];

		/* Set duty cycle limits based on current limits */
		if (currentState != STATE_CALIBRATING)
		{
			min_duty = cal_duty - 0.015;
			max_duty = cal_duty + 0.015;
		} else {
			min_duty = 0.2;
			max_duty = 0.4;
		}

		uint8_t aRxBuffer[2];
		aTxRegPtr[0] = 0x02;
		aRxBuffer[0] = 0x00;
		aRxBuffer[1] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) I2C_ADD_PWR << 1U,
				(uint8_t*) aTxRegPtr, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t) I2C_ADD_PWR << 1U,
				(uint8_t*) aRxBuffer, 2, HAL_MAX_DELAY);
		double voltage = (aRxBuffer[0] << 8 | aRxBuffer[1]) * 0.00125 * 20.1 / 5.1;

		if (tuning > tuning_threshold && adc_max > vds_checking_threshold) {
			current_duty = current_duty - 0.002;
			low_vds_count_threshold = current_duty * ADC_BUF_LEN - 1;
			if (current_duty > min_duty) {
				compare1_val = current_duty * signal_period;
				compare3_val = compare1_val + signal_period / 2;
				__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
						HRTIM_COMPAREUNIT_1, compare1_val);
				__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
						HRTIM_COMPAREUNIT_3, compare3_val);
				HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A);
			} else {
				limit_reached = true;
				gAlertType = HS;
			}
		} else if (low_vds_count > low_vds_count_threshold
				&& adc_max > vds_checking_threshold) {
			current_duty = current_duty + 0.002;
			low_vds_count_threshold = current_duty * ADC_BUF_LEN - 1;
			if (current_duty < max_duty) {
				compare1_val = current_duty * signal_period;
				compare3_val = compare1_val + signal_period / 2;
				__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
						HRTIM_COMPAREUNIT_1, compare1_val);
				__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
						HRTIM_COMPAREUNIT_3, compare3_val);
				HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A);
			} else {
				limit_reached = true;
				gAlertType = DI;
			}
		} else if (currentState == STATE_CALIBRATING
				&& adc_max > vds_checking_threshold) {
			cal_duty = current_duty;
			if (voltage > 66.5) {
				if (keepCalState == 0) {
					handleEvent(EVENT_CALIBRATE_DONE);
				} else {
					keepCalState--;
				}
			}

		}

		aTxRegPtr[0] = 0x00;
		aRxBuffer[0] = 0x00;
		aRxBuffer[1] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) I2C_ADD_TEMP1 << 1,
				(uint8_t*) aTxRegPtr, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t) I2C_ADD_TEMP1 << 1,
				(uint8_t*) aRxBuffer, 2, HAL_MAX_DELAY);
		double temp1 = (aRxBuffer[0] << 4 | aRxBuffer[1] >> 4) * 0.0625;

		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) I2C_ADD_TEMP2 << 1U,
				(uint8_t*) aTxRegPtr, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t) I2C_ADD_TEMP2 << 1U,
				(uint8_t*) aRxBuffer, 2, HAL_MAX_DELAY);
		double temp2 = (aRxBuffer[0] << 4 | aRxBuffer[1] >> 4) * 0.0625;

		aTxRegPtr[0] = 0x01;
		aRxBuffer[0] = 0x00;
		aRxBuffer[1] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) I2C_ADD_PWR << 1U,
				(uint8_t*) aTxRegPtr, 1, HAL_MAX_DELAY);
		HAL_I2C_Master_Receive(&hi2c1, (uint16_t) I2C_ADD_PWR << 1U,
				(uint8_t*) aRxBuffer, 2, HAL_MAX_DELAY);
		double current = (int16_t) (aRxBuffer[0] << 8 | aRxBuffer[1])
				* 0.0000025 / 0.012;


		// handle ALERTS
		if (temp1 > TEMP_LIMIT) {
			limit_reached = true;
			gAlertType = OT;
		}

		if (temp2 > TEMP_LIMIT) {
			limit_reached = true;
			gAlertType = OT;
		}

		if (voltage > VOLT_LIMIT){
			limit_reached = true;
			gAlertType = OV;
		}

		if (current > CURRENT_LIMIT) {
			limit_reached = true;
			gAlertType = OC;
		}

		if (limit_reached) {
			handleEvent(EVENT_ALERT);
			limit_reached = false;
			// reset waveform
			current_duty = initial_duty;
			compare1_val = current_duty * signal_period;
			compare3_val = compare1_val + signal_period / 2;
			low_vds_count_threshold = current_duty * ADC_BUF_LEN - 1;
			__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
					HRTIM_COMPAREUNIT_1, compare1_val);
			__HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
					HRTIM_COMPAREUNIT_3, compare3_val);
			HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_A);
		} else {
			gAlertType = NONE;
		}

		if (SEND_TIMER_FLAG && UART_READY && (currentState != STATE_CALIBRATING))
		{
			sprintf(json,
					"{\"temperature1\":%.2f,\"temperature2\":%.2f,\"duty\":%.3f,\"voltage\":%.2f,\"current\":%.2f,\"tuning\":%i,\"low_vds\":%i,\"low_vds_threshold\":%i,\"alert\":%i",
					temp1, temp2, current_duty, voltage, current, tuning,
					low_vds_count, low_vds_count_threshold, gAlertType);
			strcat(json, "}");

			HAL_UART_Transmit_IT(&huart2, (uint8_t*) json, strlen(json));
			UART_READY = false;
			SEND_TIMER_FLAG = false;
		}

		HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONVHRTIM_TRG1;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_B;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERB_PERIOD;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 604;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0xA0FF;
  pTimeBaseCfg.Mode = HRTIM_MODE_SINGLESHOT_RETRIGGERABLE;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 207;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 302;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 509;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_MASTERPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_ACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMCMP2;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP3;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 1216;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

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
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.Prescaler = 1024;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 18750;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_DATA_Pin|EN_FULL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_DATA_Pin EN_FULL_Pin */
  GPIO_InitStruct.Pin = LED_DATA_Pin|EN_FULL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void handleEvent(event_t event) {
	bool transitionFound = false;
	for (int i = 0; i < sizeof(transitionTable) / sizeof(transition_t); i++) {
		if (transitionTable[i].currentState == currentState
				&& transitionTable[i].event == event) {
			currentState = transitionTable[i].nextState;
			if (transitionTable[i].action != NULL) {
				transitionTable[i].action(); // Call the action function if it is not NULL
			}

			transitionFound = true;
			break;
		}
	}

	if (!transitionFound) {
//        printf("Invalid event in the current state\n");
	}
}

void actionTurnOn() {
	HAL_GPIO_WritePin(EN_FULL_GPIO_Port, EN_FULL_Pin, GPIO_PIN_SET);

}

void actionTurnOff() {
	HAL_GPIO_WritePin(EN_FULL_GPIO_Port, EN_FULL_Pin, GPIO_PIN_RESET);
}

void actionCalibrate() {
	keepCalState = 8;
	HAL_GPIO_WritePin(EN_FULL_GPIO_Port, EN_FULL_Pin, GPIO_PIN_SET);
}

void parse_json(const char *json) {
	cJSON *root = cJSON_Parse(json);
	if (root == NULL) {
		return;
	}

	cJSON *overtemp = cJSON_GetObjectItemCaseSensitive(root, "overtemperature");
	if (cJSON_IsNumber(overtemp) && (overtemp->valuedouble != 0))
		TEMP_LIMIT = overtemp->valuedouble;

	cJSON *overcurr = cJSON_GetObjectItemCaseSensitive(root, "overcurrent");
	if (cJSON_IsNumber(overcurr) && (overcurr->valuedouble != 0))
		CURRENT_LIMIT = overcurr->valuedouble;

	cJSON *overvolt = cJSON_GetObjectItemCaseSensitive(root, "overvoltage");
	if (cJSON_IsNumber(overvolt) && (overvolt->valuedouble != 0))
		VOLT_LIMIT = overvolt->valuedouble;

	cJSON *mode = cJSON_GetObjectItemCaseSensitive(root, "mode");
	if (cJSON_IsString(mode) && (mode->valuestring != NULL)) {
		if (strcmp(mode->valuestring, "off") == 0) {
			handleEvent(EVENT_OFF_BUTTON);
		}else if (strcmp(mode->valuestring, "deploy") == 0) {
			handleEvent(EVENT_DEPLOY_BUTTON);
		}

	cJSON_Delete(root);
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	UART_READY = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	SEND_TIMER_FLAG = true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		if (rxIndex < BUFFER_SIZE) {
			if (rxBuffer[rxIndex] == '{') {
				// Start of JSON data
				rxIndex = 0;
				jsonFlag = 1;
			}

			if (jsonFlag) {
				// Store received data in the buffer
				rxBuffer[rxIndex] =
						rxBuffer[rxIndex] == '\r' ? '\n' : rxBuffer[rxIndex]; // Replace '\r' with '\n'
				rxIndex++;

				if (rxBuffer[rxIndex - 1] == '}') {
					// End of JSON data
					rxBuffer[rxIndex] = '\0'; // Null-terminate the JSON string
					jsonFlag = 0;
					parse_json((const char*) rxBuffer);
				}
			}
		}

		HAL_UART_Receive_IT(&huart2, &rxBuffer[rxIndex], 1); // Enable UART receive interrupt again
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
	while (1) {
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