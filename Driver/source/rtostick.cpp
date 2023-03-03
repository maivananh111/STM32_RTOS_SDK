/*
 * rtostick.c
 *
 *  Created on: Jan 6, 2023
 *      Author: anh
 */

#include "stm32f4xx.h"
#include "system.h"
#include "systick.h"
#include "rcc.h"



#if (RTOS)
#include "rtostick.h"

static TIM_TypeDef *tim_tick = (TIM_TypeDef *)TIM_SYSTICK;

void tim_for_tick_init(uint32_t tick_priority){
	IRQn_Type IRQn;
	uint32_t prescaler = 0;

	if	   (tim_tick == TIM1)  IRQn = TIM1_UP_TIM10_IRQn;
	else if(tim_tick == TIM2)  IRQn = TIM2_IRQn;
	else if(tim_tick == TIM3)  IRQn = TIM3_IRQn;
	else if(tim_tick == TIM4)  IRQn = TIM4_IRQn;
	else if(tim_tick == TIM5)  IRQn = TIM5_IRQn;
	else if(tim_tick == TIM6)  IRQn = TIM6_DAC_IRQn;
	else if(tim_tick == TIM7)  IRQn = TIM7_IRQn;
	else if(tim_tick == TIM8)  IRQn = TIM8_UP_TIM13_IRQn;
	else if(tim_tick == TIM9)  IRQn = TIM1_BRK_TIM9_IRQn;
	else if(tim_tick == TIM10) IRQn = TIM1_UP_TIM10_IRQn;
	else if(tim_tick == TIM11) IRQn = TIM1_TRG_COM_TIM11_IRQn;
	else if(tim_tick == TIM12) IRQn = TIM8_BRK_TIM12_IRQn;
	else if(tim_tick == TIM13) IRQn = TIM8_UP_TIM13_IRQn;
	else 				   	   IRQn = TIM8_TRG_COM_TIM14_IRQn;

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_DisableIRQ(IRQn);


	if     (tim_tick == TIM1)  RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(tim_tick == TIM2)  RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(tim_tick == TIM3)  RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(tim_tick == TIM4)  RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(tim_tick == TIM5)  RCC -> APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(tim_tick == TIM6)  RCC -> APB1ENR |= RCC_APB1ENR_TIM6EN;
	else if(tim_tick == TIM7)  RCC -> APB1ENR |= RCC_APB1ENR_TIM7EN;
	else if(tim_tick == TIM8)  RCC -> APB2ENR |= RCC_APB2ENR_TIM8EN;
	else if(tim_tick == TIM9)  RCC -> APB2ENR |= RCC_APB2ENR_TIM9EN;
	else if(tim_tick == TIM10) RCC -> APB2ENR |= RCC_APB2ENR_TIM10EN;
	else if(tim_tick == TIM11) RCC -> APB2ENR |= RCC_APB2ENR_TIM11EN;
	else if(tim_tick == TIM12) RCC -> APB1ENR |= RCC_APB1ENR_TIM12EN;
	else if(tim_tick == TIM13) RCC -> APB1ENR |= RCC_APB1ENR_TIM13EN;
	else   				   	   RCC -> APB1ENR |= RCC_APB1ENR_TIM14EN;

	if(tim_tick == TIM1 || tim_tick == TIM8 || tim_tick == TIM9 || tim_tick == TIM10 || tim_tick == TIM11)
		prescaler = rcc_get_bus_frequency(APB2_TIMER);
	else
		prescaler = rcc_get_bus_frequency(APB1_TIMER);
	prescaler = (uint32_t) ((prescaler / 1000000U) - 1U);

	tim_tick -> SR = 0;
	tim_tick -> PSC = prescaler;
	tim_tick -> ARR = (1000000U / SYSTICK_RATE) - 1U;
	tim_tick -> EGR = TIM_EGR_UG;

	tim_tick -> DIER |= TIM_DIER_UIE;
	tim_tick -> CR1 |= TIM_CR1_CEN;
	tim_tick -> SR = 0;


	NVIC_Set_Priority(IRQn, 0U, 0U);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);
}

extern "C"{
	void TIM_TICK_IRQ_HANDLER(void){
		tim_tick -> SR =~ TIM_SR_UIF;
		increment_tick();
	}
}

#endif
