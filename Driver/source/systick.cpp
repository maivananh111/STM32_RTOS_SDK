/*
 * systick.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "systick.h"
#include "sdkconfig.h"
#include "system.h"

#include "stdio.h"



volatile uint32_t tick;

static void (*delay_ms_func)(uint32_t) = systick_delay_ms;
static uint32_t (*get_tick_func)(void) = systick_get_tick;



#if (!RTOS)
void systick_init(uint32_t systick_priority){
	SysTick_Config(SystemCoreClock / SYSTICK_RATE);

	NVIC_Set_Priority(SysTick_IRQn, systick_priority, 0U);
}
#endif

void increment_tick(void){
	tick++;
}

uint32_t systick_get_tick(void){
	return tick;
}

void systick_delay_ms(uint32_t ms){
	uint32_t tickstart = tick;
	uint32_t wait = ms;

	if (wait < 0xFFFFFFU) wait += 1UL;

	while((tick - tickstart) < wait);
}

uint32_t get_tick(void){
	return get_tick_func();
}

void delay_ms(uint32_t ms){
	delay_ms_func(ms);
}

void set_function_get_tick(uint32_t (*func_ptr)(void)){
	get_tick_func = func_ptr;
}

void set_function_delay_ms(void(*func_ptr)(uint32_t)){
	delay_ms_func = func_ptr;
}


#if (!RTOS)
extern "C"{
	void SysTick_Handler(void){
		increment_tick();
	}
}
#endif













