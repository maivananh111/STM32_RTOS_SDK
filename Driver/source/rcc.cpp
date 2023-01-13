/*
 * rcc.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */


#include "rcc.h"

#include "sdkconfig.h"
#include "system.h"
#include "systick.h"
#include "stdio.h"
#include "math.h"

#if(RTOS)
#include "rtostick.h"
#endif

#define RCC_HSI_TIMEOUT          100U
#define RCC_HSE_TIMEOUT          200U
#define RCC_PLL_TIMEOUT          100U
#define RCC_SWS_TIMEOUT 	 	 5000U

static RCC_Config_t *_conf;

Result_t rcc_init(RCC_Config_t *rcc_conf){
#if (!OVER_CLOCK)
#if(SYSTEM_CLOCK_FREQUENCY > SYSTEM_CLOCK_FREQUENCY_MAX)
#error "SYSTEM_CLOCK_FREQUENCY out of range. Modify System clock frequency less than or equal to SYSTEM_CLOCK_FREQUENCY_MAX in sdkconfig.h file."
#endif
#if(AHB_CLOCK_FREQUENCY > AHB_CLOCK_FREQUENCY_MAX)
#error "AHB_CLOCK_FREQUENCY out of range. Modify AHB clock frequency less than or equal to AHB_CLOCK_FREQUENCY_MAX in sdkconfig.h file."
#endif
#endif
#if(APB1_CLOCK_FREQUENCY > APB1_CLOCK_FREQUENCY_MAX)
#error "APB1_CLOCK_FREQUENCY out of range. Modify APB1 clock frequency less than or equal to APB1_CLOCK_FREQUENCY_MAX in sdkconfig.h file."
#endif
#if(APB2_CLOCK_FREQUENCY > APB2_CLOCK_FREQUENCY_MAX)
#error "APB2_CLOCK_FREQUENCY out of range. Modify APB2 clock frequency less than or equal to APB2_CLOCK_FREQUENCY_MAX in sdkconfig.h file."
#endif

	Result_t res = {OKE, 0U};
	__IO uint32_t tmpreg = 0;
	_conf = rcc_conf;

	/**
	 * OSC Configuration.
	 */
	if((RCC -> CFGR & RCC_CFGR_SWS_HSE) || ((RCC -> CFGR & RCC_CFGR_SWS_PLL) && (RCC -> PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE))){
		if(!(RCC -> CR & RCC_CR_HSERDY)){
			set_result(&res, ERR, __LINE__);
			return res;
		}
	}

	if(_conf -> osc_source == HSI_CRYSTAL){
		RCC -> CR |= RCC_CR_HSION;
		res = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_HSIRDY, FLAG_SET, RCC_HSI_TIMEOUT);
		if(result_is_timeout(&res)) {
			set_result_line(&res, __LINE__);
			return res;
		}
		RCC -> CR &= ~RCC_CR_HSITRIM_Msk;
		RCC -> CR |= (_conf -> hsi_trim << RCC_CR_HSITRIM_Pos);

	}
	else if(_conf -> osc_source == HSE_CRYSTAL){
		RCC -> CR |= RCC_CR_HSEON;
		res = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_HSERDY, FLAG_SET, RCC_HSE_TIMEOUT);
		if(result_is_timeout(&res)) {
			set_result_line(&res, __LINE__);
			return res;
		}
	}
	else{
		set_result(&res, ERR, __LINE__);
		return res;
	}

	/**
	 * PLL Configuration.
	 */
	if(_conf -> sysclock_source == PLLCLK){
		if(!(RCC -> CFGR & RCC_CFGR_SWS_PLL)){
			RCC -> CR &=~ RCC_CR_PLLON;
			res = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_RESET, RCC_PLL_TIMEOUT);
			if(result_is_timeout(&res)) {
				set_result_line(&res, __LINE__);
				return res;
			}

			tmpreg = RCC -> PLLCFGR;
			tmpreg &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLSRC_Msk);
			tmpreg |= ((_conf -> pll.pllm) << RCC_PLLCFGR_PLLM_Pos) | ((_conf -> pll.plln) << RCC_PLLCFGR_PLLN_Pos) | ((((_conf -> pll.pllp) >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) |
					  ((_conf -> pll.pllq) << RCC_PLLCFGR_PLLQ_Pos) | ((_conf -> pll_source) << RCC_PLLCFGR_PLLSRC_Pos);
			RCC -> PLLCFGR = tmpreg;

			RCC -> CR |= RCC_CR_PLLON;
			res = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_SET, RCC_PLL_TIMEOUT);
			if(result_is_timeout(&res)) {
				set_result_line(&res, __LINE__);
				return res;
			}
		}
	}


	/**
	 * Calculation flash latency and update latency if new latency great than current latency.
	 */
	uint32_t latency = embedded_flash_calculate_latency(AHB_CLOCK_FREQUENCY);
	uint32_t current_latency = embedded_flash_get_latency();
	if(latency > current_latency) embedded_flash_set_latency(latency);

	/**
	 * Check and set system clock source.
	 */
	if(_conf -> sysclock_source == HSI){
		if(!(RCC -> CR & RCC_CR_HSIRDY)){
			set_result_line(&res, __LINE__);
			return res;
		}
	}
	else if(_conf -> sysclock_source == HSE){
		if(!(RCC -> CR & RCC_CR_HSERDY)){
			set_result_line(&res, __LINE__);
			return res;
		}
	}
	else if(_conf -> sysclock_source == PLLCLK){
		if(!(RCC -> CR & RCC_CR_PLLRDY)){
			set_result_line(&res, __LINE__);
			return res;
		}
	}

	RCC -> CFGR = ((RCC -> CFGR & !RCC_CFGR_SW_Msk) | (_conf -> sysclock_source << RCC_CFGR_SW_Pos));
	res = wait_flag_in_register_timeout(&(RCC -> CFGR), (_conf -> sysclock_source << RCC_CFGR_SW_Pos), FLAG_SET, RCC_SWS_TIMEOUT);
	if(result_is_timeout(&res)) {
		set_result_line(&res, __LINE__);
		return res;
	}

	/**
	 * SYSCLK, AHB, APB1, APB2 frequency configuration.
	 */
	tmpreg = RCC -> CFGR;
	tmpreg &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
	tmpreg |= (_conf -> ahb_prescaler) | (_conf -> apb1_prescaler) | (_conf -> apb2_prescaler);
	RCC -> CFGR = tmpreg;

	/**
	 * Update system core clock..
	 */
	SystemCoreClockUpdate();

	/**
	 * Update latency if new latency less than current latency.
	 */
	if(latency < current_latency) embedded_flash_set_latency(latency);

	/**
	 * ReInit system tick.
	 */
#if (!RTOS)
	systick_init(SYSTICK_PRIORITY);
#else
	tim_for_tick_init(SYSTICK_PRIORITY);
#endif

	return res;
}

Result_t rcc_deinit(void){
	Result_t res = {OKE, 0U};

	/**
	 * Turn on HSI (default).
	 */
	RCC -> CR |= RCC_CR_HSION;
	res = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_HSIRDY, FLAG_SET, RCC_HSI_TIMEOUT);
	if(result_is_timeout(&res)) {
		set_result_line(&res, __LINE__);
		return res;
	}
	RCC -> CR &= ~RCC_CR_HSITRIM_Msk;
	RCC -> CR |= (0x10U << RCC_CR_HSITRIM_Pos);

	/**
	 * Clear CFGR register.
	 */
	RCC -> CFGR = 0x00U;
	res = wait_flag_in_register_timeout(&(RCC -> CFGR), 0xFFFFFFFFU, FLAG_RESET, RCC_SWS_TIMEOUT);
	if(result_is_timeout(&res)) {
		set_result_line(&res, __LINE__);
		return res;
	}

	/**
	 * Turn off HSE.
	 */
	RCC -> CR &=~ RCC_CR_HSEON;
	res = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_HSERDY, FLAG_RESET, RCC_HSE_TIMEOUT);
	if(result_is_timeout(&res)) {
		set_result_line(&res, __LINE__);
		return res;
	}

	/**
	 * Turn off PLL.
	 */
	RCC -> CR &=~ RCC_CR_PLLON;
	res = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_RESET, RCC_PLL_TIMEOUT);
	if(result_is_timeout(&res)) {
		set_result_line(&res, __LINE__);
		return res;
	}

	RCC -> PLLCFGR = RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_2;

	SystemCoreClock = HSI_VALUE;

	/**
	 * ReInit system tick.
	 */
#if (!RTOS)
	systick_init(SYSTICK_PRIORITY);
#else
	tim_for_tick_init(SYSTICK_PRIORITY);
#endif

	return res;
}

uint32_t rcc_get_bus_frequency(Bus_Clock_t bus){
	switch(bus){
		case SYSCLK:
			if(_conf -> osc_source == HSE_CRYSTAL){ // HSE.
				if(_conf -> sysclock_source == HSE) return (uint32_t)HSE_VALUE;
				else if(_conf -> sysclock_source == PLLCLK) return (uint32_t)(((HSE_VALUE / _conf -> pll.pllm) * _conf -> pll.plln) / _conf -> pll.pllp);
			}
			else{ // HSI.
				if(_conf -> sysclock_source == HSI) return (uint32_t)HSI_VALUE;
				else if(_conf -> sysclock_source == PLLCLK) return (uint32_t)(((HSE_VALUE / _conf -> pll.pllm) * _conf -> pll.plln) / _conf -> pll.pllp);
			}
		break;

		case AHB:
			if(_conf -> ahb_prescaler <= 7) return (uint32_t)SystemCoreClock;
			return (uint32_t)(SystemCoreClock / (uint32_t)abs((int)(_conf -> ahb_prescaler - 6U)));
		break;

		case APB1:
			return (uint32_t)(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
		break;

		case APB2:
			return (uint32_t)(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
		break;

		case APB1_TIMER:
			return (uint32_t)(2*(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]));
		break;

		case APB2_TIMER:
			return (uint32_t)(2*(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]));
		break;

	}
	return 0;
}



































