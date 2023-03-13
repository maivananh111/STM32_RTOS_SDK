/*
 * system.c
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "system.h"
#include "sdkconfig.h"
#include "systick.h"

#include "stdio.h"
#include "malloc.h"



extern "C" char *sbrk(int i);
extern char _end;
extern char _sdata;
extern char _estack;
extern char _Min_Stack_Size;

static char *ramstart = &_sdata;
static char *ramend = &_estack;
static char *minSP = (char*)(ramend - &_Min_Stack_Size);

uint32_t get_revid(void){
	return((DBGMCU -> IDCODE) >> 16U);
}

uint32_t get_devid(void){
	return((DBGMCU -> IDCODE) & 0x0FFFU);
}

void system_init(void){
	embedded_flash_init();

	__NVIC_SetPriorityGrouping(0x03U);

	systick_init(SYSTICK_PRIORITY);

	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC -> APB1ENR |= RCC_APB1ENR_PWREN;
}

void embedded_flash_init(void){
#if FLASH_INSTRUCTION_CACHE
	FLASH -> ACR |= FLASH_ACR_ICEN;
#else
	FLASH -> ACR &=~ FLASH_ACR_ICEN;
#endif

#if FLASH_DATA_CACHE
	FLASH -> ACR |= FLASH_ACR_DCEN;

#else
	FLASH -> ACR &=~ FLASH_ACR_DCEN;
#endif

#if FLASH_PREFETCH_MODE
	FLASH -> ACR |= FLASH_ACR_PRFTEN;
#else
	FLASH -> ACR &=~ FLASH_ACR_PRFTEN;
#endif

}

void embedded_flash_set_latency(uint32_t latency){
	if(latency > FLASH_ACR_LATENCY_7WS) latency = FLASH_ACR_LATENCY_7WS;

	FLASH -> ACR = ((FLASH -> ACR & (~FLASH_ACR_LATENCY_Msk)) | (latency << FLASH_ACR_LATENCY_Pos));
}

void embedded_flash_update_latency(void){
	uint32_t tmpreg = (FLASH -> ACR & (~FLASH_ACR_LATENCY_Msk));

	uint32_t latency= (uint32_t)(SystemCoreClock / 30000000U);
	if(SystemCoreClock == 30000000U || SystemCoreClock == 60000000U || SystemCoreClock == 90000000U
    || SystemCoreClock == 120000000U || SystemCoreClock == 150000000U || SystemCoreClock == 180000000U) latency -= 1;

	tmpreg |= (uint32_t)(latency << FLASH_ACR_LATENCY_Pos);
	FLASH -> ACR |= tmpreg;
}

uint32_t embedded_flash_calculate_latency(uint32_t freq){
	uint32_t latency= (uint32_t)(freq / 30000000U);
	if(freq == 30000000U || freq == 60000000U || freq == 90000000U
    || freq == 120000000U || freq == 150000000U || freq == 180000000U) latency -= 1;

	return latency;
}

uint32_t embedded_flash_get_latency(void){
	return (FLASH -> ACR & FLASH_ACR_LATENCY_Msk >> FLASH_ACR_LATENCY_Pos);
}


void NVIC_Set_Priority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority){
	uint32_t prioritygroup = 0x00U;

	if(PreemptPriority > 15U) PreemptPriority = 15U;
	if(SubPriority > 15U) SubPriority = 15U;

	prioritygroup = __NVIC_GetPriorityGrouping();

	__NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}

memory_info_t stm_get_memory_info(void){
	memory_info_t mem;
	char *heapend = (char*)sbrk(0);
	char * stack_ptr = (char*)__get_MSP();
	struct mallinfo mi = mallinfo();

	mem.free_ram = ((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks;
	mem.heap_ram_used = mi.uordblks;
	mem.prog_ram_used = &_end - ramstart;
	mem.stack_ram_used = ramend - stack_ptr;
	mem.total_free_ram = mi.fordblks;

	return mem;
}

uint32_t stm_get_free_heap_size(void){
	char *heapend = (char*)sbrk(0);
	char * stack_ptr = (char*)__get_MSP();
	struct mallinfo mi = mallinfo();

	return ((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks;
}

uint32_t stm_get_used_heap_size(void){
	struct mallinfo mi = mallinfo();

	return mi.uordblks;
}

extern "C"{
void NMI_Handler(void){
	__NVIC_SystemReset();
}

void HardFault_Handler(void){
	__NVIC_SystemReset();
}

void MemManage_Handler(void){
	__NVIC_SystemReset();
}

void BusFault_Handler(void){
	__NVIC_SystemReset();
}

void UsageFault_Handler(void){
	__NVIC_SystemReset();
}

}

























