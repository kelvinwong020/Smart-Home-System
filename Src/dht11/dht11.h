#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f1xx_hal.h"

typedef struct
{
	uint8_t presence;
	uint8_t  humi_int;		
	uint8_t  humi_deci;	 	
	uint8_t  temp_int;	 	
	uint8_t  temp_deci;	 
	uint8_t  check_sum;	 	
		                 
} DHT11_Data_TypeDef;

#define DHT11_PORT GPIOE
#define DHT11_PIN GPIO_PIN_6

extern TIM_HandleTypeDef htim1;

void delay_us(uint16_t us);

void DHT11_Start(void);

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

int8_t DHT11_Check_Response(void);

uint8_t DHT11_Read(void);

int8_t DHT11_Read_TempAndHumidity(DHT11_Data_TypeDef *DHT11_Data);

uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi);

uint8_t DHT11_Init(void);

#endif /*__DHT11_*/
