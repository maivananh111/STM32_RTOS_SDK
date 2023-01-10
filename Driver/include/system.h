/*
 * system.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "stm32f4xx.h"


#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	Scale3 = 1,
	Scale2,
	scale1,
} power_voltage_regulator_scale_t;

typedef struct{
	uint32_t heap_ram_used;
	uint32_t prog_ram_used;
	uint32_t stack_ram_used;
	uint32_t free_ram;
	uint32_t total_free_ram;
} Memory_Info_t;

uint32_t get_revid(void);
uint32_t get_devid(void);

void system_init(void);

void embedded_flash_init(void);
void embedded_flash_set_latency(uint32_t latency);
uint32_t embedded_flash_calculate_latency(uint32_t freq);
uint32_t embedded_flash_get_latency(void);
void embedded_flash_update_latency(void);

void NVIC_Set_Priority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);

Memory_Info_t get_memory_info(void);
uint32_t get_free_heap_size(void);
uint32_t get_used_heap_size(void);


#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_H_ */
