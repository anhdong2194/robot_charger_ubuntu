/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define ADC_Option_Pin GPIO_PIN_3
#define ADC_Option_GPIO_Port GPIOC
#define Battery_Pin GPIO_PIN_0
#define Battery_GPIO_Port GPIOA
#define Temp_Sensor_Pin GPIO_PIN_1
#define Temp_Sensor_GPIO_Port GPIOA
#define IR_Receive_Pin GPIO_PIN_6
#define IR_Receive_GPIO_Port GPIOA
#define IR_Receive_BK_Pin GPIO_PIN_7
#define IR_Receive_BK_GPIO_Port GPIOA
#define Charge_Pin GPIO_PIN_0
#define Charge_GPIO_Port GPIOB
#define Forward_LMS_Pin GPIO_PIN_1
#define Forward_LMS_GPIO_Port GPIOB
#define Backward_LMS_Pin GPIO_PIN_2
#define Backward_LMS_GPIO_Port GPIOB
#define Forward_Motor_Pin GPIO_PIN_10
#define Forward_Motor_GPIO_Port GPIOB
#define Backward_Motor_Pin GPIO_PIN_11
#define Backward_Motor_GPIO_Port GPIOB
#define Charge_LMS_Pin GPIO_PIN_12
#define Charge_LMS_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_13
#define Buzzer_GPIO_Port GPIOB
#define Operator_Light_Pin GPIO_PIN_14
#define Operator_Light_GPIO_Port GPIOB
#define Error_Light_Pin GPIO_PIN_15
#define Error_Light_GPIO_Port GPIOB
#define Charge_Finish_Pin GPIO_PIN_6
#define Charge_Finish_GPIO_Port GPIOC
#define IR_Transmit_Pin GPIO_PIN_8
#define IR_Transmit_GPIO_Port GPIOA
#define USART_Wifi_Tx_Pin GPIO_PIN_9
#define USART_Wifi_Tx_GPIO_Port GPIOA
#define USART_Wifi_Rx_Pin GPIO_PIN_10
#define USART_Wifi_Rx_GPIO_Port GPIOA
#define Batt_lvl_1_Pin GPIO_PIN_3
#define Batt_lvl_1_GPIO_Port GPIOB
#define Batt_lvl_2_Pin GPIO_PIN_4
#define Batt_lvl_2_GPIO_Port GPIOB
#define Batt_lvl_3_Pin GPIO_PIN_5
#define Batt_lvl_3_GPIO_Port GPIOB
#define Batt_lvl_4_Pin GPIO_PIN_6
#define Batt_lvl_4_GPIO_Port GPIOB
#define Batt_lvl_5_Pin GPIO_PIN_7
#define Batt_lvl_5_GPIO_Port GPIOB
#define Option_Pin GPIO_PIN_8
#define Option_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
