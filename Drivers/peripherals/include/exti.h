/*
 * exti.h
 *
 *  Created on: Jan 10, 2023
 *      Author: anh
 */

#ifndef EXTI_H_
#define EXTI_H_

#include "peripheral_enable.h"

#if ENABLE_EXTI

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "status.h"


typedef enum{
	EXTI_RISING_EDGE  = EXTI_RTSR_TR0,
	EXTI_FALLING_EDGE = EXTI_FTSR_TR0,
	EXTI_RISING_FALLING_EDGE  = EXTI_RTSR_TR0 | EXTI_FTSR_TR0,
} exti_edgedetect_t;

return_t exti_init(GPIO_TypeDef *Port, uint16_t Pin, exti_edgedetect_t Edge, uint32_t Priority);
void exti_deinit(GPIO_TypeDef *Port, uint16_t Pin);

void exti_register_event_handler(void (*function_ptr)(uint16_t pin, void *param), void *param);
void exti_unregister_event_handler(void);


#ifdef ENABLE_EXTI0
void EXTI0_IRQHandler(void);             			/* EXTI Line0 interrupt */
#endif /* ENABLE_EXTI0 */
#ifdef ENABLE_EXTI1
void EXTI1_IRQHandler(void);              			/* EXTI Line1 interrupt */
#endif /* ENABLE_EXTI1 */
#ifdef ENABLE_EXTI2
void EXTI2_IRQHandler(void);              			/* EXTI Line2 interrupt */
#endif /* ENABLE_EXTI2 */
#ifdef ENABLE_EXTI3
void EXTI3_IRQHandler(void);              			/* EXTI Line3 interrupt */
#endif /* ENABLE_EXTI3 */
#ifdef ENABLE_EXTI4
void EXTI4_IRQHandler(void);              			/* EXTI Line4 interrupt */
#endif /* ENABLE_EXTI4 */
#ifdef ENABLE_EXTI9_5
void EXTI9_5_IRQHandler(void);             			/* EXTI Line[9:5] interrupts */
#endif /* ENABLE_EXTI9_5 */
#ifdef ENABLE_EXTI15_10
void EXTI15_10_IRQHandler(void);         			/* EXTI Line[15:10] interrupts */
#endif /* ENABLE_EXTI15_10 */



#ifdef __cplusplus
}
#endif

#endif

#endif /* EXTI_H_ */
