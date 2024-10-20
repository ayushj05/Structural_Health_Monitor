/*
 * dht11.h
 *
 *  Created on: Mar 27, 2024
 *      Author: ayush
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "stm32f1xx_hal_conf.h"

#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9

#define DHT11_TIMEOUT 2000

//#define SYSTICKCLOCK 72000000ULL
//#define SYSTICKPERUS (SYSTICKCLOCK / 1000000UL)
//
//// delay has to constant expression
//static void inline __attribute__((always_inline)) microDelay(uint16_t delay)
//{
//    uint32_t ticks = 72 * delay;
//    uint32_t start_tick = SysTick -> VAL;
//
//    while(SysTick -> VAL - start_tick < ticks);
//}

extern TIM_HandleTypeDef htim1, htim2;

uint32_t pMillis, cMillis;

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT11_Start (void)
{
	uint8_t Response = 0;
	GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
	GPIO_InitStructPrivate.Pin = DHT11_PIN;
	GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	HAL_Delay(20);   // wait for 20ms
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	microDelay (30);   // wait for 30us
	GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
	microDelay (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		microDelay (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
			Response = 1;
	}
	pMillis = HAL_GetTick();
	cMillis = HAL_GetTick();
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
	{
		cMillis = HAL_GetTick();
	}
	return Response;
}

uint8_t DHT11_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

#endif /* INC_DHT11_H_ */
