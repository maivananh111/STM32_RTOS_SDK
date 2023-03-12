/*
 * systick.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_


#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C"{
#endif

extern volatile uint32_t tick;

void systick_init(uint32_t systick_priority);

void increment_tick(void);

uint32_t systick_get_tick(void);
void systick_delay_ms(uint32_t ms);

uint32_t get_tick(void);
void delay_ms(uint32_t ms);

void set_function_get_tick(uint32_t (*func_ptr)(void));
void set_function_delay_ms(void(*func_ptr)(uint32_t));

void SysTick_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* SYSTICK_H_ */
