/*
 * exti.cpp
 *
 *  Created on: Jan 10, 2023
 *      Author: anh
 */

#include "periph_en.h"

#ifdef ENABLE_EXTI

#include "exti.h"
#include "stdio.h"
#include "system.h"

#define EXTI_LINE_INDEX 6U

void (*handler_callback)(uint16_t pin, void *param) = NULL;
void *parameter = NULL;

extern "C"{void EXTI_IRQHandler(uint16_t Pin);}

void exti_init(GPIO_TypeDef *Port, uint16_t Pin, EXTI_EdgeDetect_t Edge, uint32_t Priority){
	uint8_t CRPos = 0;
	IRQn_Type IRQn;

	if(Pin < 4U) 					CRPos = 0;
	else if(Pin >= 4U && Pin < 8U)  CRPos = 1;
	else if(Pin >= 8U && Pin < 12U) CRPos = 2;
	else 							CRPos = 3;

	if(Pin < 5U) IRQn = (IRQn_Type)(Pin + EXTI_LINE_INDEX);
	else if(Pin >= 5U && Pin < 9U) IRQn = EXTI9_5_IRQn;
	else 						   IRQn = EXTI15_10_IRQn;

	if(!(RCC -> APB2ENR & RCC_APB2ENR_SYSCFGEN)) RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;


	__IO uint32_t tmpreg = SYSCFG -> EXTICR[CRPos];

	tmpreg &=~ (0x0F << ((Pin - CRPos*4U) * 4U));
	tmpreg |= (uint32_t)(((((uint32_t)Port & 0xFF00U) >> 8U) / 4U) << ((Pin - CRPos*4U) * 4U));

	SYSCFG -> EXTICR[CRPos] = tmpreg;

	if(Edge & EXTI_RTSR_TR0) EXTI -> RTSR |= (1U << Pin);
	if(Edge & EXTI_FTSR_TR0) EXTI -> FTSR |= (1U << Pin);

	EXTI -> IMR |= (1U << Pin);

	__NVIC_SetPriority(IRQn, Priority);
	__NVIC_EnableIRQ(IRQn);
}

void exti_deinit(GPIO_TypeDef *Port, uint16_t Pin){
	uint8_t CRPos = 0;
	if(Pin < 4U) 					CRPos = 0;
	else if(Pin >= 4U && Pin < 8U)  CRPos = 1;
	else if(Pin >= 8U && Pin < 12U) CRPos = 2;
	else 							CRPos = 3;
	EXTI -> PR = (1U << Pin);

	SYSCFG -> EXTICR[CRPos] &=~ (0x0F << ((Pin - CRPos*4U) * 4U));

	EXTI -> RTSR &=~ (1U << Pin);
	EXTI -> FTSR &=~ (1U << Pin);

	EXTI -> IMR &=~ (1U << Pin);

	IRQn_Type IRQn;
	if(Pin < 5U) IRQn = (IRQn_Type)(Pin + EXTI_LINE_INDEX);
	else if(Pin >= 5U && Pin < 9U) IRQn = EXTI9_5_IRQn;
	else 						   IRQn = EXTI15_10_IRQn;
	__NVIC_DisableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);
}

void exti_register_event_handler(void (*function_ptr)(uint16_t pin, void *param), void *param){
	handler_callback = function_ptr;
	parameter = param;
}

void exti_unregister_event_handler(void){
	handler_callback = NULL;
}

extern "C"{
void EXTI_IRQHandler(uint16_t Pin){
	if(EXTI -> PR & (1U << Pin)){
		EXTI -> PR = (1U << Pin);
		if(handler_callback != NULL) handler_callback(Pin, parameter);
	}
}


#ifdef ENABLE_EXTI0
void EXTI0_IRQHandler(void){
	EXTI_IRQHandler(0);
}
#endif
#ifdef ENABLE_EXTI1
void EXTI1_IRQHandler(void){
	EXTI_IRQHandler(1);
}
#endif
#ifdef ENABLE_EXTI2
void EXTI2_IRQHandler(void){
	EXTI_IRQHandler(2);
}
#endif
#ifdef ENABLE_EXTI3
void EXTI3_IRQHandler(void){
	EXTI_IRQHandler(3);
}
#endif
#ifdef ENABLE_EXTI4
void EXTI4_IRQHandler(void){
	EXTI_IRQHandler(4);
}
#endif
#ifdef ENABLE_EXTI9_5
void EXTI9_5_IRQHandler(void){
	EXTI_IRQHandler(5);
	EXTI_IRQHandler(6);
	EXTI_IRQHandler(7);
	EXTI_IRQHandler(8);
	EXTI_IRQHandler(9);
}
#endif
#ifdef ENABLE_EXTI15_10
void EXTI15_10_IRQHandler(void){
	EXTI_IRQHandler(10);
	EXTI_IRQHandler(11);
	EXTI_IRQHandler(12);
	EXTI_IRQHandler(13);
	EXTI_IRQHandler(14);
	EXTI_IRQHandler(15);
}
#endif
}

#endif











