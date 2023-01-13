/*
 * rcc.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef RCC_H_
#define RCC_H_

#include "stm32f4xx.h"
#include "status.h"

#ifdef __cplusplus
extern "C"{
#endif


typedef enum{
	HSI_CRYSTAL,
	HSE_CRYSTAL,
} OSC_Source_t;

typedef enum{
	PLL_SOURCE_HSI,
	PLL_SOURCE_HSE,
} PLL_Source_t;

typedef enum{
	HSI,
	HSE,
	PLLCLK,
} SysClock_Source_t;

typedef enum{
	SYSCLK,
	AHB,
	APB1,
	APB2,
	APB1_TIMER,
	APB2_TIMER,
} Bus_Clock_t;

typedef struct{
	uint32_t hse_frequency;
	uint32_t hsi_frequency;
	uint32_t hsi_trim;
	OSC_Source_t osc_source;
	SysClock_Source_t sysclock_source;
	PLL_Source_t pll_source;
	uint32_t sysclock_frequency;
	uint32_t ahb_prescaler;
	uint32_t apb1_prescaler;
	uint32_t apb2_prescaler;
	struct{
		uint32_t pllm;
		uint32_t plln;
		uint32_t pllp;
		uint32_t pllq;
	} pll;
}RCC_Config_t;

Result_t rcc_init(RCC_Config_t *rcc_conf);
Result_t rcc_deinit(void);

uint32_t rcc_get_bus_frequency(Bus_Clock_t bus);



#ifdef __cplusplus
}
#endif

#endif /* RCC_H_ */
