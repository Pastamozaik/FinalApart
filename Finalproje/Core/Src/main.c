#include "stm32f4xx.h"
#include "stm32f4xx_hal_rcc.h"

uint16_t adcValue;

void ADC1_Init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  GPIOA->MODER |= GPIO_MODER_MODER0;
  ADC1->CR1 &= (ADC_CR1_SCAN);
  ADC1->CR2 |= ~ADC_CR2_CONT;

  uint32_t clockFreq = HAL_RCC_GetPCLK2Freq();
  uint32_t sampleTime = 2 * (clockFreq / 10);
  ADC1->SMPR2 |= sampleTime << ADC_SMPR2_SMP0_Pos;
  ADC1->CR2 |= ADC_CR2_ADON;
}

void TIM4_PWM_Init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  GPIOB->MODER &= ~(GPIO_MODER_MODER7);
  GPIOB->MODER |= GPIO_MODER_MODER7_1;
  GPIOB->AFR[0] |= (2 << (7 * 4));

  uint32_t clockFreq = SystemCoreClock;
  uint32_t desiredFreq = 1000;
  uint32_t psc = clockFreq / (desiredFreq * 10000);
  uint32_t arr = clockFreq / (desiredFreq * psc);

  TIM4->PSC = 65500;
  TIM4->ARR = 10*arr - 1;
  TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
  TIM4->CCER |= TIM_CCER_CC1E;
  TIM4->CCR1 = (uint32_t)(arr * 0.95);
  TIM4->CR1 |= TIM_CR1_CEN;
}

void Read_ADC_Value(void)
{
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while (!(ADC1->SR & ADC_SR_EOC));
  adcValue = ADC1->DR;
}

int main(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  ADC1_Init();
  TIM4_PWM_Init();

  while (1)
  {
    Read_ADC_Value();
  }
}
