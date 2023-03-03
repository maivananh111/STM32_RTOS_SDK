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
} rcc_oscsource_t;

typedef enum{
	PLL_SOURCE_HSI,
	PLL_SOURCE_HSE,
} rcc_pllsource_t;

typedef enum{
	HSI,
	HSE,
	PLLCLK,
} rcc_sysclocksource_t;

typedef enum{
	SYSCLK,
	AHB,
	APB1,
	APB2,
	APB1_TIMER,
	APB2_TIMER,
} rcc_busclock_t;

typedef enum{
	Clock_Div_1 = 0U,
	Clock_Div_2,
	Clock_Div_4,
	Clock_Div_8,
	Clock_Div_16,
	Clock_Div_32,
	Clock_Div_64,
	Clock_Div_128,
	Clock_Div_256,
	Clock_Div_512,
} rcc_clockdiv_t;

typedef struct{
	uint32_t hse_frequency 				= 25000000U;
	uint32_t hsi_frequency 				= 16000000U;
	uint32_t hsi_trim					= 16;
	rcc_oscsource_t osc_source				= HSI_CRYSTAL;
	rcc_sysclocksource_t sysclock_source	= HSI;
	rcc_pllsource_t pll_source				= PLL_SOURCE_HSI;
	uint32_t sysclock_frequency			= 16000000U;
	rcc_clockdiv_t ahb_prescaler;
	rcc_clockdiv_t apb1_prescaler;
	rcc_clockdiv_t apb2_prescaler;
	struct{
		uint32_t pllm;
		uint32_t plln;
		uint32_t pllp;
		uint32_t pllq;
	} pll;
}RCC_Config_t;

return_t rcc_init(RCC_Config_t *rcc_conf);
return_t rcc_deinit(void);

uint32_t rcc_get_bus_frequency(rcc_busclock_t bus);



#ifdef __cplusplus
}
#endif

#endif /* RCC_H_ */
