/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f30x.h"

void systickInit(uint16_t frequency)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  (void) SysTick_Config (RCC_Clocks.HCLK_Frequency / frequency);
}

volatile uint32_t ticks;
inline uint32_t millis (void)
{
  return ticks;
}

void delay_ms (uint32_t t)
{
  uint32_t start,end;
  start = millis();
  end = start + t;
  if (start < end) {
    while ((millis() >= start) && (millis() < end)){
      //nothing
    }
  }else{
    while((millis() >= start) || (millis() < end)){
      //nothing
    }
  }
}

void timer_toggle_led (uint32_t t, uint8_t led_status)
{
  uint32_t start,end;
  start = millis();
  end = start + t;

  if (start > end) {

  }
}

int main(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOA, &gpio);

    systickInit(1024);

    GPIO_SetBits(GPIOA, GPIO_Pin_5);

    while (1)
    {
      // Nothing
    }
}


void SysTick_Handler (void)
{
  ticks++;

  uint32_t led_period_ms = 500;
  if(!(ticks % led_period_ms)){
    if(GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5)){
      GPIO_ResetBits(GPIOA, GPIO_Pin_5); // LED OFF
    }else{
      GPIO_SetBits(GPIOA, GPIO_Pin_5); // LED ON
    }
  }
}
