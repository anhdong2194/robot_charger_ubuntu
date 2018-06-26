/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "math.h"

#define State_1  0
#define State_2  1
#define State_3	 2

#define Not_yet   0
#define Full		 1

#define Station_ID 	  	1000
#define Waiting 				6000
#define Charge_Start 		6001
#define Charging 	    	6002
#define Reset_Workflow 	6003

#define Charge_waiting 			7000
#define Charge_motor_error 	7001
#define Charge_charging			7002
#define Charge_full					7003
#define Charge_prepare			7004
#define Charge_reset				7005

#define Charge_Enable HAL_GPIO_WritePin(GPIOB,Charge_Pin,GPIO_PIN_SET);
#define Charge_Disable HAL_GPIO_WritePin(GPIOB,Charge_Pin,GPIO_PIN_RESET);

#define Operator_On HAL_GPIO_WritePin(GPIOB,Operator_Light_Pin,GPIO_PIN_SET);
#define Operator_Off HAL_GPIO_WritePin(GPIOB,Operator_Light_Pin,GPIO_PIN_RESET);

#define Error_On HAL_GPIO_WritePin(GPIOB,Error_Light_Pin,GPIO_PIN_SET);
#define Error_Off HAL_GPIO_WritePin(GPIOB,Error_Light_Pin,GPIO_PIN_RESET);

#define Buzzer_On HAL_GPIO_WritePin(GPIOB,Buzzer_Pin,GPIO_PIN_SET);
#define Buzzer_Off HAL_GPIO_WritePin(GPIOB,Buzzer_Pin,GPIO_PIN_RESET);


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
void get_data(void);
void battery_indicator(void);

/*------------ Timer delay -------------*/
void delay_us(uint16_t microseconds);

void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Off(void);
/* Private function prototypes -----------------------------------------------*/

//ADC
//================================================================================//
volatile uint16_t adc1_value[3];
int adc1_value_temp = 0;
float adc1_value_batt = 0;
float NTC_value = 0;
float min_diff = 1000;

//-20 => 105
float Temp_lookup_table[126] = {\
		 70.5811, 67.2987, 64.1834, 61.2233, 58.4080, 55.7284, 53.1766, 50.7456, 48.4294, 46.2224, 44.1201, 42.1180, 40.2121\
		,38.3988, 36.6746, 35.0362, 33.4802, 32.0035, 30.6028, 29.2750, 28.0170, 26.8255, 25.6972, 24.6290, 23.6176, 22.6597\
		,21.7522, 20.8916, 20.0749, 19.2988, 18.5600, 18.4818, 18.1489, 17.6316, 16.9917, 16.2797, 15.5350, 14.7867, 14.0551\
		,13.3536, 12.6900, 12.0684, 11.4900, 10.9539, 10.4582, 10.0000, 9.5762,  9.1835,  8.8186,  8.4784,  8.1600,  7.8608\
		,7.5785,  7.3109,  7.0564,  6.8133,  6.5806,  6.3570,  6.1418,  5.9343,  5.7340,  5.5405,  5.3534,  5.1725,  4.9976\
		,4.8286,  4.6652,  4.5073,  4.3548,  4.2075,  4.0650,	 3.9271,  3.7936,  3.6639,  3.5377,  3.4146,  3.2939,  3.1752\
	  ,3.0579,  2.9414,  2.8250,  2.7762,  2.7179,  2.6523,  2.5817,  2.5076,  2.4319,  2.3557,  2.2803,  2.2065,  2.1350\
    ,2.0661,  2.0004,  1.9378,  1.8785,  1.8225,  1.7696,  1.7197,  1.6727,  1.6282,  1.5860,  1.5458,  1.5075,  1.4707\
	  ,1.4352,  1.4006,  1.3669,  1.3337,  1.3009,  1.2684,  1.2360,  1.2037,  1.1714,  1.1390,  1.1067,  1.0744,  1.0422\
    ,1.0104,  0.9789,  0.9481,  0.9180,  0.8889,  0.85,    0.8346,  0.8099,  0.7870};

//UART WIFI
//================================================================================//
uint8_t buffer[100],receive_data[6];
uint8_t data_complete = 0, data_len = 0;
uint16_t data_id = 0;
uint8_t len = 0;
uint8_t data_order = 0, data_order_2 = 0;
uint8_t data_reorder[6];

//WORKFLOW
//================================================================================//
uint8_t workflow = 0, lms_ = 1, wait_motor = 0, charge_complete = 0;

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();

	HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start_IT(&htim3);
	//---Battery LED---
	HAL_GPIO_WritePin(GPIOB,Batt_lvl_1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,Batt_lvl_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,Batt_lvl_3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,Batt_lvl_4_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,Batt_lvl_5_Pin,GPIO_PIN_SET);

	//---Error Light---//0 = off
	Error_Off;
	//---Operator Light---//0 = off
	Operator_Off;
	//---Buzzer---//0 = off
	Buzzer_Off;

	//---Charge---//0 = off
	Charge_Disable;
	//---Motor---//0 = off
	Motor_Off();

	//---UART DMA---
	len = sizeof(receive_data)/sizeof(receive_data[0]);
	HAL_UART_Receive_DMA(&huart1,receive_data,len);

	//---ADC DMA---
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) adc1_value,3);
	HAL_Delay(1000);

  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//Get DMA data
		get_data();

		//Auto restart if temp too high
		if (adc1_value_temp >= 90){
						snprintf((char*)buffer, sizeof buffer\
						,"%s%d%s%d%s%d%s%.1f%s"\
						,"{\"id\":",Station_ID,",\"status\":",Charge_reset,",\"temp\":",adc1_value_temp,",\"batt\":",adc1_value_batt,"}\n");//r
						HAL_UART_Transmit_IT(&huart1,buffer, sizeof buffer);
						Charge_Disable;
						Motor_Backward();
						while(wait_motor <= 100){
							delay_us(50000);
							wait_motor++;
						}
						for (int i=0; i<len; i++) receive_data[i] = '\0';
						data_complete = 0;
						workflow = State_1;
						Error_On;
						Operator_Off;
		}

		//Response to server while waiting
		if ((data_complete == 1)&&(data_id == Waiting)&&(workflow == State_1)){
			snprintf((char*)buffer, sizeof buffer\
				,"%s%d%s%d%s%d%s%.1f%s"\
				,"{\"id\":",Station_ID,",\"status\":",Charge_waiting,",\"temp\":",adc1_value_temp,",\"batt\":",adc1_value_batt,"}\n");//r
			HAL_UART_Transmit_IT(&huart1,buffer, sizeof buffer);
			for (int i=0; i<len; i++) receive_data[i] = '\0';
			data_complete = 0;
			//charge_complete = 0;
			Error_Off;
			Operator_On;
		}
		//Response to server while starting to charge
		else if ((data_complete == 1)&&(data_id == Charge_Start)&&(workflow == State_1)){
			Motor_Forward();
			//HAL_Delay(500);
			snprintf((char*)buffer, sizeof buffer\
				,"%s%d%s%d%s%d%s%.1f%s"\
				,"{\"id\":",Station_ID,",\"status\":",Charge_prepare,",\"temp\":",adc1_value_temp,",\"batt\":",adc1_value_batt,"}\n");//r
			HAL_UART_Transmit_IT(&huart1,buffer, sizeof buffer);
			wait_motor = 0;
			while(wait_motor <= 100){
				delay_us(50000);
				wait_motor++;
			}

			if (HAL_GPIO_ReadPin(GPIOB,Charge_LMS_Pin) == 1){
				lms_ = 1;
				workflow = State_3;
				Error_On;
				Operator_Off;
			}
			else if (HAL_GPIO_ReadPin(GPIOB,Charge_LMS_Pin) == 0){
				lms_ = 0;
				workflow = State_2;
				Charge_Enable;
				Error_Off;
			  Operator_On;
			}

			Motor_Off();
			data_complete = 0;
			for (int i=0; i<len; i++) receive_data[i] = '\0';
		}
		//Response to server while motor error
		else if ((data_complete == 1)&&(data_id == Charging)&&(workflow == State_3)){
			snprintf((char*)buffer, sizeof buffer\
				,"%s%d%s%d%s%d%s%.1f%s"\
				,"{\"id\":",Station_ID,",\"status\":",Charge_motor_error,",\"temp\":",adc1_value_temp,",\"batt\":",adc1_value_batt,"}\n");//r
			HAL_UART_Transmit_IT(&huart1,buffer, sizeof buffer);
			for (int i=0; i<len; i++) receive_data[i] = '\0';
			data_complete = 0;
		}
		//Response to server while charging
		else if ((data_complete == 1)&&(data_id == Charging)&&(workflow == State_2)){
				snprintf((char*)buffer, sizeof buffer\
				,"%s%d%s%d%s%d%s%.1f%s"\
				,"{\"id\":",Station_ID,",\"status\":",Charge_charging,",\"temp\":",adc1_value_temp,",\"batt\":",adc1_value_batt,"}\n");//r

			HAL_UART_Transmit_IT(&huart1,buffer, sizeof buffer);
			for (int i=0; i<len; i++) receive_data[i] = '\0';
			data_complete = 0;
		}
		//Reset charge station workflow
		else if ((data_complete == 1)&&(data_id == Reset_Workflow)&&(workflow != State_1)){
			snprintf((char*)buffer, sizeof buffer\
				,"%s%d%s%d%s%d%s%.1f%s"\
				,"{\"id\":",Station_ID,",\"status\":",Charge_reset,",\"temp\":",adc1_value_temp,",\"batt\":",adc1_value_batt,"}\n");//r
			HAL_UART_Transmit_IT(&huart1,buffer, sizeof buffer);
			Charge_Disable;
			Motor_Backward();
			while(wait_motor <= 100){
				delay_us(50000);
				wait_motor++;
			}
			for (int i=0; i<len; i++) receive_data[i] = '\0';
			data_complete = 0;
			workflow = State_1;
			Error_Off;
			Operator_On;
		}

		//Full condition
		if ((workflow == State_2)&&(adc1_value_batt >= 29.3)){
			Charge_Disable;
			snprintf((char*)buffer, sizeof buffer\
				,"%s%d%s%d%s%d%s%.1f%s"\
				,"{\"id\":",Station_ID,",\"status\":",Charge_full,",\"temp\":",adc1_value_temp,",\"batt\":",adc1_value_batt,"}\n");//r
			HAL_UART_Transmit_IT(&huart1,buffer, sizeof buffer);
			for (int i=0; i<len; i++) receive_data[i] = '\0';
			data_complete = 0;

			delay_us(5000);
			Motor_Backward();
			wait_motor = 0;
			while(wait_motor <= 100){
				delay_us(50000);
				wait_motor++;
			}
		}

		//while charging show battery lvl
		if (workflow == State_2)
			battery_indicator();
		else if(workflow != State_2){
			HAL_GPIO_WritePin(GPIOB,Batt_lvl_1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,Batt_lvl_2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,Batt_lvl_3_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,Batt_lvl_4_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,Batt_lvl_5_Pin,GPIO_PIN_SET);
		}

  }
}

/** System Clock Configuration
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==htim3.Instance)
	{
		//Battery calc
		adc1_value_batt = adc1_value[0]*0.007531f;

		//Temp of charge station
		NTC_value = (10.89 * adc1_value[1])/(13513.5 - (3.3 * adc1_value[1]));
		min_diff = 1000;
		for (int i = 0; i<126; i++){
			if (fabs(Temp_lookup_table[i] - NTC_value) <= min_diff){
				min_diff = fabs(Temp_lookup_table[i] - NTC_value);
				adc1_value_temp = i - 20;
			}
		}
	}
}

void battery_indicator(void){
	//adc1_value_batt = adc1_value[0]*0.007531f;

	if (adc1_value_batt < 26){
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_2_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_4_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_5_Pin,GPIO_PIN_SET);
	}
	else if (adc1_value_batt < 27){
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_3_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_4_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_5_Pin,GPIO_PIN_SET);
	}
	else if (adc1_value_batt < 28){
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_4_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_5_Pin,GPIO_PIN_SET);
	}
	else if (adc1_value_batt < 29){
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_4_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_5_Pin,GPIO_PIN_SET);
	}
	else if (adc1_value_batt >= 29.3){
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_2_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_3_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_4_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB,Batt_lvl_5_Pin,GPIO_PIN_RESET);
	}
}

void Motor_Forward(void){
	HAL_GPIO_WritePin(GPIOB,Forward_Motor_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,Backward_Motor_Pin,GPIO_PIN_RESET);
}

void Motor_Backward(void){
	HAL_GPIO_WritePin(GPIOB,Forward_Motor_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,Backward_Motor_Pin,GPIO_PIN_SET);
}

void Motor_Off(void){
	HAL_GPIO_WritePin(GPIOB,Forward_Motor_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,Backward_Motor_Pin,GPIO_PIN_RESET);
}

void get_data(void)
{
		//find the order of data
		for (int i = 0; i<len; i++){
			if (receive_data[i] == 'c')
				data_order = i;
		}

		if (data_order != 0){
			for (int i = 0; i<len; i++){
				data_reorder[i] = receive_data[data_order];
				data_order_2 = i+1;
				data_order++;
				if (data_order == len)
					break;
			}
			for (int i = 0; i<len; i++){
				data_reorder[data_order_2] = receive_data[i];
				data_order_2++;
				if (data_order_2 == len)
					break;
			}
		}
		else{
			for (int i = 0; i<len; i++){
				data_reorder[i] = receive_data[i];
			}
		}

		//find the start and stop
		if ((data_reorder[0] == 'c')&&(data_reorder[5] == 'e')){
			data_complete = 1;
			uint16_t dec = 1000;
			data_id = 0;
			for (int i = 1; i<len; i++){
				if (data_reorder[i] == 'e') break;
				data_id = data_id + ((data_reorder[i] - '0')*dec);
				dec = dec/10;
			}
		}
}

/*--------------- Delay US ------------------*/
void delay_us(uint16_t microseconds)
{
	TIM4->CNT = 0;
	//volatile uint16_t start = TIM1->CNT;
	while((TIM4->CNT) < microseconds);
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 599;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IR_Receive_BK_GPIO_Port, IR_Receive_BK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Charge_Pin|Forward_Motor_Pin|Backward_Motor_Pin|Buzzer_Pin
                          |Operator_Light_Pin|Error_Light_Pin|Batt_lvl_1_Pin|Batt_lvl_2_Pin
                          |Batt_lvl_3_Pin|Batt_lvl_4_Pin|Batt_lvl_5_Pin|Option_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IR_Receive_Pin */
  GPIO_InitStruct.Pin = IR_Receive_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_Receive_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_Receive_BK_Pin */
  GPIO_InitStruct.Pin = IR_Receive_BK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IR_Receive_BK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Charge_Pin Forward_Motor_Pin Backward_Motor_Pin Buzzer_Pin
                           Operator_Light_Pin Error_Light_Pin Batt_lvl_1_Pin Batt_lvl_2_Pin
                           Batt_lvl_3_Pin Batt_lvl_4_Pin Batt_lvl_5_Pin Option_Pin */
  GPIO_InitStruct.Pin = Charge_Pin|Forward_Motor_Pin|Backward_Motor_Pin|Buzzer_Pin
                          |Operator_Light_Pin|Error_Light_Pin|Batt_lvl_1_Pin|Batt_lvl_2_Pin
                          |Batt_lvl_3_Pin|Batt_lvl_4_Pin|Batt_lvl_5_Pin|Option_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Forward_LMS_Pin Backward_LMS_Pin Charge_LMS_Pin */
  GPIO_InitStruct.Pin = Forward_LMS_Pin|Backward_LMS_Pin|Charge_LMS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Charge_Finish_Pin */
  GPIO_InitStruct.Pin = Charge_Finish_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Charge_Finish_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
