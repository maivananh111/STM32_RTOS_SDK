/*
 * tim.cpp
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */


#include "periph_en.h"
#ifdef ENABLE_TIM

#include "rcc.h"
#include "gpio.h"
#include "tim.h"
#include "stm_log.h"

#define UIF_TIMEOUT 100U


static const char *TAG = "TIM";
TIM::TIM(TIM_TypeDef *Timer){
	_tim = Timer;
}

/* TIM Basic */
return_t TIM::init(tim_config_t *conf){
	return_t ret;

	_conf = conf;

	if     (_tim == TIM1)  RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(_tim == TIM2)  RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(_tim == TIM3)  RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(_tim == TIM4)  RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(_tim == TIM5)  RCC -> APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(_tim == TIM6)  RCC -> APB1ENR |= RCC_APB1ENR_TIM6EN;
	else if(_tim == TIM7)  RCC -> APB1ENR |= RCC_APB1ENR_TIM7EN;
	else if(_tim == TIM8)  RCC -> APB2ENR |= RCC_APB2ENR_TIM8EN;
	else if(_tim == TIM9)  RCC -> APB2ENR |= RCC_APB2ENR_TIM9EN;
	else if(_tim == TIM10) RCC -> APB2ENR |= RCC_APB2ENR_TIM10EN;
	else if(_tim == TIM11) RCC -> APB2ENR |= RCC_APB2ENR_TIM11EN;
	else if(_tim == TIM12) RCC -> APB1ENR |= RCC_APB1ENR_TIM12EN;
	else if(_tim == TIM13) RCC -> APB1ENR |= RCC_APB1ENR_TIM13EN;
	else   				   RCC -> APB1ENR |= RCC_APB1ENR_TIM14EN;

	/* BASIC TIMER */
	_tim -> CR1 = 0U;
	_tim -> CR1 |= (_conf -> direction << TIM_CR1_DIR_Pos) | (_conf -> autoreloadpreload << TIM_CR1_ARPE_Pos) | (_conf -> align << TIM_CR1_CMS_Pos);

	_tim -> ARR = 0U;
	_tim -> ARR = _conf -> reload - 1;
	_tim -> PSC = 0U;
	_tim -> PSC = _conf -> prescaler - 1;

	_tim -> EGR = TIM_EGR_UG;

	if(_conf -> interrupt == TIM_INTERRUPT_ENABLE){
		if	   (_tim == TIM1)  IRQn = TIM1_UP_TIM10_IRQn;
		else if(_tim == TIM2)  IRQn = TIM2_IRQn;
		else if(_tim == TIM3)  IRQn = TIM3_IRQn;
		else if(_tim == TIM4)  IRQn = TIM4_IRQn;
		else if(_tim == TIM5)  IRQn = TIM5_IRQn;
		else if(_tim == TIM6)  IRQn = TIM6_DAC_IRQn;
		else if(_tim == TIM7)  IRQn = TIM7_IRQn;
		else if(_tim == TIM8)  IRQn = TIM8_UP_TIM13_IRQn;
		else if(_tim == TIM9)  IRQn = TIM1_BRK_TIM9_IRQn;
		else if(_tim == TIM10) IRQn = TIM1_UP_TIM10_IRQn;
		else if(_tim == TIM11) IRQn = TIM1_TRG_COM_TIM11_IRQn;
		else if(_tim == TIM12) IRQn = TIM8_BRK_TIM12_IRQn;
		else if(_tim == TIM13) IRQn = TIM8_UP_TIM13_IRQn;
		else 				   IRQn = TIM8_TRG_COM_TIM14_IRQn;
	}

	return ret;
}

tim_config_t *TIM::get_config(void){
	return _conf;
}

void TIM::set_prescaler (uint32_t psc){
	_conf -> prescaler = psc;
	_tim -> PSC = psc;
}
void TIM::set_autoreload(uint32_t arl){
	_conf -> reload = arl;
	_tim -> ARR = arl;
}

void TIM::reset_counter(void){
	_tim -> CNT = 0;
}

uint16_t TIM::get_counter(void){
	return _tim -> CNT;
}

void TIM::delay_us(uint32_t us){
	_tim -> CNT = 0;
	while(_tim -> CNT < us);
}

void TIM::delay_ms(uint32_t ms){
	for (uint32_t i=0; i<ms; i++) delay_us(1000);
}

void TIM::clear_update_isr(void){
	_tim -> SR &=~ TIM_DIER_UIE;
}


return_t TIM::register_event_handler(void(*function_ptr)(tim_channel_t channel, tim_event_t event, void *param), void *param){
	return_t ret;

	if(_conf -> interrupt != TIM_INTERRUPT_ENABLE) {
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disabled, can't register timer event handler.", __FILE__, __FUNCTION__);
		return ret;
	}

	parameter = param;
	handler_callback = function_ptr;

	return ret;
}

return_t TIM::unregister_event_handler(void){
	return_t ret;

	if(handler_callback == NULL){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt wasn't register event handler, can't unregister.", __FILE__, __FUNCTION__);
		return ret;
	}

	handler_callback = NULL;

	return ret;
}

return_t TIM::start(void){
	return_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, BUSY, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::stop(void){
	return_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, ERR, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}

return_t TIM::start_it(void){
	return_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
		return ret;
	}

#ifdef RTOS
	if(_conf -> interruptpriority < RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
		return ret;
	}
#endif

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, BUSY, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	_tim -> DIER |= TIM_DIER_UIE;
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::stop_it(void){
	return_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
		return ret;
	}

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, ERR, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer not started, can't stop in interrupt mode.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> CR1 &=~ TIM_CR1_CEN;

	if(_conf -> interrupt == TIM_INTERRUPT_ENABLE){
		 _tim -> DIER &=~ TIM_DIER_UIE;
		__NVIC_ClearPendingIRQ(IRQn);
		__NVIC_DisableIRQ(IRQn);
	}

	return ret;
}

#ifdef ENABLE_DMA
return_t TIM::start_dma(uint32_t *count_buf, uint16_t size){
	return_t ret;

	if(_conf -> dma_upd == NULL){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer not set dma, can't start in dma mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	_tim -> CR1  &=~ TIM_CR1_CEN;

	ret = _conf -> dma_upd -> start((uint32_t)count_buf, (uint32_t)&_tim -> CNT, size);
	if(!is_oke(&ret)) {
		set_return_line(&ret, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
		return ret;
	}

	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::stop_dma(void){
	return_t ret;

	if(_conf -> dma_upd == NULL){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer not set dma, can't stop in dma mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	if((_tim -> DIER & TIM_DIER_CC1DE) && (_tim -> CR1 & TIM_CR1_CEN)){
		ret = _conf -> dma_upd -> stop();
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
			STM_LOGE(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
			return ret;
		}

		_tim -> DIER &=~ TIM_DIER_CC1DE;
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer dma not start.", __FILE__, __FUNCTION__);
	}

	return ret;
}
#endif

/* TIMER PWM OUTPUT MODE */
return_t TIM::set_mode_pwm_output(tim_channel_t channel, tim_pwm_t *conf){
	return_t ret;

	gpio_port_clock_enable(conf->port);

	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
	else {
		set_return(&ret, NOTAVAILABLE, __LINE__);
		return ret;
	}
	gpio_set_alternatefunction(conf->port, conf->pin, func);
	gpio_set_alternatefunction_type(conf->port, conf->pin, GPIO_OUTPUT_PUSHPULL);

	if(channel < TIM_CHANNEL3){ // Channel 1-2
		_tim -> CCMR1 &=~ (0xFF << (channel*8));
		_tim -> CCMR1 &=~ (TIM_CCMR1_CC1S << (channel*8));
		_tim -> CCMR1 |= ((conf->preload << TIM_CCMR1_OC1PE_Pos) << (channel*8));
		_tim -> CCMR1 |= ((conf->fastmode << TIM_CCMR1_OC1FE_Pos) << (channel*8));
		_tim -> CCMR1 |= ((conf->invert << TIM_CCMR1_OC1M_Pos) << (channel*8));
	}
	else if(channel >= TIM_CHANNEL3 && channel < TIM_NOCHANNEL){ // Channel 3-4
		_tim -> CCMR2 &=~ (0xFF << ((channel-2)*8));
		_tim -> CCMR2 &=~ (TIM_CCMR2_CC3S << ((channel-2)*8));
		_tim -> CCMR2 |= ((conf->preload << TIM_CCMR2_OC3PE_Pos) << ((channel - 2)*8));
		_tim -> CCMR2 |= ((conf->fastmode << TIM_CCMR2_OC3FE_Pos) << ((channel - 2)*8));
		_tim -> CCMR2 |= ((conf->invert << TIM_CCMR2_OC3M_Pos) << ((channel - 2)*8));
	}

	return ret;
}

return_t TIM::pwm_output_start(tim_channel_t channel, uint32_t pwm){
	return_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, BUSY, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	pwm_output_set_duty(channel, pwm);

	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::pwm_output_stop(tim_channel_t channel){
	return_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, ERR, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
		return ret;
	}

	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}


return_t TIM::pwm_output_start_it(tim_channel_t channel, uint32_t pwm){
	return_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
		return ret;
	}

#ifdef RTOS
	if(_conf -> interruptpriority < RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
		return ret;
	}
#endif

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, BUSY, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	pwm_output_set_duty(channel, pwm);

	_tim -> DIER |= (TIM_DIER_CC1IE << channel);
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::pwm_output_stop_it(tim_channel_t channel){
	return_t ret;
	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
		return ret;
	}

	if((_tim -> DIER & TIM_DIER_CC1IE << channel) && (_tim -> CR1 & TIM_CR1_CEN)){
		_tim -> DIER &=~ (TIM_DIER_CC1IE << channel);
		_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
		_tim -> CR1 &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, ERR, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer not started, can't stop in interrupt mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_DisableIRQ(IRQn);

	return ret;
}

#ifdef ENABLE_DMA
return_t TIM::pwm_output_start_dma(tim_channel_t channel, uint32_t *pwm_buffer, uint16_t size){
	return_t ret;
	dma_t dma = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dma = _conf -> dma_ch1;
		break;
		case TIM_CHANNEL2:
			dma = _conf -> dma_ch2;
		break;
		case TIM_CHANNEL3:
			dma = _conf -> dma_ch3;
		break;
		case TIM_CHANNEL4:
			dma = _conf -> dma_ch4;
		break;
		default:
		break;
	};

	if(dma == NULL){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer not set dma, can't start in dma mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_tim -> DIER &=~ TIM_DIER_CC1DE << channel;
	_tim -> CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	switch(channel){
		case TIM_CHANNEL1:
			CCRx_addr = (uint32_t)&_tim -> CCR1;
		break;
		case TIM_CHANNEL2:
			CCRx_addr = (uint32_t)&_tim -> CCR2;
		break;
		case TIM_CHANNEL3:
			CCRx_addr = (uint32_t)&_tim -> CCR3;
		break;
		case TIM_CHANNEL4:
			CCRx_addr = (uint32_t)&_tim -> CCR4;
		break;
		default:
		break;
	};
	ret = dma -> start((uint32_t)pwm_buffer, CCRx_addr, size);
	if(!is_oke(&ret)) {
		set_return_line(&ret, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> DIER |= TIM_DIER_CC1DE << channel;
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::pwm_output_stop_dma(tim_channel_t channel){
	return_t ret;
	dma_t dma = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dma = _conf -> dma_ch1;
		break;
		case TIM_CHANNEL2:
			dma = _conf -> dma_ch2;
		break;
		case TIM_CHANNEL3:
			dma = _conf -> dma_ch3;
		break;
		case TIM_CHANNEL4:
			dma = _conf -> dma_ch4;
		break;
		default:
		break;
	};

	if(dma == NULL){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer not set dma, can't stop in dma mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	if((_tim -> DIER & TIM_DIER_CC1DE << channel) && (_tim -> CR1 & TIM_CR1_CEN)){
		_tim -> DIER &=~ (TIM_DIER_CC1DE << channel);
		ret = dma -> stop();
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
			STM_LOGE(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
			return ret;
		}
		_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer dma not started.", __FILE__, __FUNCTION__);
	}

	return ret;
}
#endif

return_t TIM::pwm_output_set_duty(tim_channel_t channel, uint32_t pwm){
	return_t ret;

	switch(channel){
		case TIM_CHANNEL1:
			_tim -> CCR1 = pwm;
		break;
		case TIM_CHANNEL2:
			_tim -> CCR2 = pwm;
		break;
		case TIM_CHANNEL3:
			_tim -> CCR3 = pwm;
		break;
		case TIM_CHANNEL4:
			_tim -> CCR4 = pwm;
		break;
		default:
		break;
	};

	return ret;
}


/* TIMER PWM INPUT MODE */
return_t TIM::set_mode_pwm_input(tim_channel_t channel, tim_pwm_t *conf){
	return_t ret;

	if(channel == TIM_CHANNEL3 || channel == TIM_CHANNEL4) {
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer PWM input unsupported on channel3 and channel 4.", __FILE__, __FUNCTION__);
		return ret;
	}
	if(conf -> polarity == TIM_BOTH_EDGE){
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer PWM input unsupported both edge.", __FILE__, __FUNCTION__);
		return ret;
	}

	gpio_port_clock_enable(conf->port);

	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
	else {
		set_return(&ret, NOTAVAILABLE, __LINE__);
		return ret;
	}
	gpio_set_alternatefunction(conf->port, conf->pin, func);
	gpio_set_alternatefunction_type(conf->port, conf->pin, GPIO_OUTPUT_PUSHPULL);

	_tim -> CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);

	_tim -> SMCR &=~ TIM_SMCR_TS_Msk;
	_tim -> SMCR |= ((channel+5U) << TIM_SMCR_TS_Pos) | TIM_SMCR_SMS_2;

	_tim -> CCMR1 &=~ (TIM_CCMR1_IC1F_Msk << (channel*8));
	_tim -> CCMR1 |= (conf -> ch1_filter << TIM_CCMR1_IC1F_Pos);
	_tim -> CCMR1 |= (conf -> ch2_filter << TIM_CCMR1_IC2F_Pos);

	_tim -> CCER &=~ (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);
	if(channel == TIM_CHANNEL1){
		_tim -> CCER |= (conf -> polarity << TIM_CCER_CC1P_Pos);
		_tim -> CCER |= (!(conf -> polarity) << TIM_CCER_CC2P_Pos);
	}
	else{
		_tim -> CCER |= (!(conf -> polarity) << TIM_CCER_CC1P_Pos);
		_tim -> CCER |= (conf -> polarity << TIM_CCER_CC2P_Pos);
	}

	_tim -> CCMR1 &=~ (TIM_CCMR1_CC1S_Msk << (channel*8U));
	_tim -> CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;

	return ret;
}

return_t TIM::pwm_input_start(tim_channel_t channel){
	return inputcapture_start(channel);
}

return_t TIM::pwm_input_stop(tim_channel_t channel){
	return inputcapture_stop(channel);
}

return_t TIM::pwm_input_start_it(tim_channel_t channel){
	return inputcapture_start_it(channel);
}

return_t TIM::pwm_input_stop_it(tim_channel_t channel){
	return inputcapture_stop_it(channel);
}
#ifdef ENABLE_DMA
return_t TIM::pwm_input_start_dma(tim_channel_t channel, uint32_t *pwm_buffer, uint16_t size){
	return inputcapture_start_dma(channel, pwm_buffer, size);
}

return_t TIM::pwm_input_stop_dma(tim_channel_t channel){
	return inputcapture_stop_dma(channel);
}
#endif




/* TIMER ENCODER MODE */
return_t TIM::set_mode_encoder(tim_encoder_t *conf){
	return_t ret;

	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
	else {
		set_return(&ret, UNSUPPORTED, __LINE__);
		return ret;
	}

	if	   (_tim == TIM1)  IRQn = TIM1_CC_IRQn;
	else if(_tim == TIM8)  IRQn = TIM8_CC_IRQn;

	gpio_port_clock_enable(conf -> encA_ch1_port);
	gpio_port_clock_enable(conf -> encB_ch2_port);

	gpio_set_alternatefunction(conf -> encA_ch1_port, conf -> encA_ch1_pin, func);
	gpio_set_alternatefunction(conf -> encB_ch2_port, conf -> encB_ch2_pin, func);

	gpio_set_alternatefunction_type(conf -> encA_ch1_port, conf -> encA_ch1_pin, GPIO_OUTPUT_PUSHPULL);
	gpio_set_alternatefunction_type(conf -> encB_ch2_port, conf -> encB_ch2_pin, GPIO_OUTPUT_PUSHPULL);

	_tim -> SMCR &=~ TIM_SMCR_SMS;
	_tim -> SMCR |= (conf -> mode << TIM_SMCR_SMS_Pos);

	_tim -> CCMR1 = 0U;
	_tim -> CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	_tim -> CCMR1 |= ((conf -> encA_ch1_prescaler << TIM_CCMR1_IC1PSC_Pos) | (conf -> encB_ch2_prescaler << TIM_CCMR1_IC2PSC_Pos));
	_tim -> CCMR1 |= ((conf -> encA_ch1_filter << TIM_CCMR1_IC1F_Pos) | (conf -> encB_ch2_filter << TIM_CCMR1_IC2F_Pos));

	_tim -> CCER &=~ (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);
	_tim -> CCER |= ((conf -> encA_ch1_edge << TIM_CCER_CC1P_Pos) | (conf -> encB_ch2_edge << TIM_CCER_CC2P_Pos));

	return ret;
}

return_t TIM::encoder_start(void){
	return_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, BUSY, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart.", __FILE__, __FUNCTION__);
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_tim -> CCER |= TIM_CCER_CC1E;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_tim -> CCER |= TIM_CCER_CC2E;
	}
	else{
		_tim -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	}

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::encoder_stop(void){
	return_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, ERR, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_tim -> CCER &=~ TIM_CCER_CC1E;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_tim -> CCER &=~ TIM_CCER_CC2E;
	}
	else{
		_tim -> CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);
	}

	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}

return_t TIM::encoder_start_it(void){
	return_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, NOTAVAILABLE, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
		return ret;
	}

#ifdef RTOS
	if(_conf -> interruptpriority < RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
		return ret;
	}
#endif

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, BUSY, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_tim -> CCER |= TIM_CCER_CC1E;
		_tim -> DIER |= TIM_DIER_CC1IE;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_tim -> CCER |= TIM_CCER_CC2E;
		_tim -> DIER |= TIM_DIER_CC2IE;
	}
	else{
		_tim -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
		_tim -> DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;
	}

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::encoder_stop_it(void){
	return_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, NOTAVAILABLE, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
		return ret;
	}

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, ERR, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> CR1 &=~ TIM_CR1_CEN;
	_tim -> DIER &=~ TIM_DIER_UIE;
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_DisableIRQ(IRQn);

	return ret;
}

#ifdef ENABLE_DMA
return_t TIM::encoder_start_dma(uint32_t *encA_buffer, uint32_t *encB_buffer, uint16_t size){
	return_t ret;

	if((_conf -> dma_ch1 == NULL && _conf -> dma_ch2 == NULL) || (encA_buffer == NULL && encB_buffer == NULL)){
		set_return(&ret, NOTAVAILABLE, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer not set dma or data buffer, can't start in dma mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	_tim -> CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	switch(mode){
		case TIM_ENCODER_MODE1:
			_tim -> DIER &=~ TIM_DIER_CC1DE;
			_tim -> CCER &=~ TIM_CCER_CC1E;

			CCRx_addr = (uint32_t)&_tim -> CCR1;
			ret = _conf -> dma_ch1 -> start(CCRx_addr, (uint32_t)encA_buffer, size);
			if(!is_oke(&ret)) {
				set_return_line(&ret, __LINE__);
				STM_LOGE(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
				return ret;
			}

			_tim -> DIER |= TIM_DIER_CC1DE;
			_tim -> CCER |= TIM_CCER_CC1E;
		break;

		case TIM_ENCODER_MODE2:
			_tim -> DIER &=~ TIM_DIER_CC2DE;
			_tim -> CCER &=~ TIM_CCER_CC2E;

			CCRx_addr = (uint32_t)&_tim -> CCR2;
			ret = _conf -> dma_ch2 -> start(CCRx_addr, (uint32_t)encB_buffer, size);
			if(!is_oke(&ret)) {
				set_return_line(&ret, __LINE__);
				STM_LOGE(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
				return ret;
			}

			_tim -> DIER |= TIM_DIER_CC2DE;
			_tim -> CCER |= TIM_CCER_CC2E;
		break;

		case TIM_ENCODER_MODE3:
			_tim -> DIER &=~ (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
			_tim -> CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);

			CCRx_addr = (uint32_t)&_tim -> CCR1;
			ret = _conf -> dma_ch1 -> start(CCRx_addr, (uint32_t)encA_buffer, size);
			if(!is_oke(&ret)) {
				set_return_line(&ret, __LINE__);
				STM_LOGE(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
				return ret;
			}

			CCRx_addr = (uint32_t)&_tim -> CCR2;
			ret = _conf -> dma_ch2 -> start(CCRx_addr, (uint32_t)encB_buffer, size);
			if(!is_oke(&ret)) {
				set_return_line(&ret, __LINE__);
				STM_LOGE(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
				return ret;
			}

			_tim -> DIER |= (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
			_tim -> CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
		break;

		default:
		break;
	};
	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::encoder_stop_dma(void){
	return_t ret;

	if((_conf -> dma_ch1 == NULL && _conf -> dma_ch2 == NULL)){
		set_return(&ret, NOTAVAILABLE, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer not set dma or data buffer, can't stop in dma mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	if(((_tim -> DIER & TIM_DIER_CC1DE) || (_tim -> DIER & TIM_DIER_CC2DE)) && (_tim -> CR1 & TIM_CR1_CEN)){
		tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
		switch(mode){
			case TIM_ENCODER_MODE1:
				_tim -> DIER &=~ TIM_DIER_CC1DE;
				ret = _conf -> dma_ch1 -> stop();
				if(!is_oke(&ret)) {
					set_return_line(&ret, __LINE__);
					STM_LOGE(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
					return ret;
				}
				_tim -> CCER &=~ TIM_CCER_CC1E;
			break;

			case TIM_ENCODER_MODE2:
				_tim -> DIER &=~ TIM_DIER_CC2DE;
				ret = _conf -> dma_ch2 -> stop();
				if(!is_oke(&ret)) {
					set_return_line(&ret, __LINE__);
					STM_LOGE(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
					return ret;
				}
				_tim -> CCER &=~ TIM_CCER_CC2E;
			break;

			case TIM_ENCODER_MODE3:
				_tim -> DIER &=~ (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
				ret = _conf -> dma_ch1 -> stop();
				if(!is_oke(&ret)) {
					set_return_line(&ret, __LINE__);
					STM_LOGE(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
					return ret;
				}
				ret = _conf -> dma_ch2 -> stop();
				if(!is_oke(&ret)) {
					set_return_line(&ret, __LINE__);
					STM_LOGE(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
					return ret;
				}
				_tim -> CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);
			break;

			default:
			break;
		};
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer dma not started.", __FILE__, __FUNCTION__);
	}

	return ret;
}
#endif

int16_t TIM::encoder_get_counter(void){
	return (int16_t)((int16_t)counter/4);
}

uint32_t TIM::encoder_get_base_counter(void){
	return counter;
}


/* TIMER INPUT CAPTURE MODE */
return_t TIM::set_mode_inputcapture(tim_channel_t channel, tim_inputcapture_t *conf){
	return_t ret;

	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
	else {
		set_return(&ret, UNSUPPORTED, __LINE__);
		return ret;
	}

	if	   (_tim == TIM1)  IRQn = TIM1_CC_IRQn;
	else if(_tim == TIM8)  IRQn = TIM8_CC_IRQn;

	gpio_port_clock_enable(conf -> port);
	gpio_set_alternatefunction(conf -> port, conf -> pin, func);
	gpio_set_alternatefunction_type(conf -> port, conf -> pin, GPIO_OUTPUT_PUSHPULL);

	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4U));
	if(channel < TIM_CHANNEL3){
		_tim -> CCMR1 &=~ (0xFF << (channel));
		_tim -> CCMR1 |= (TIM_CCMR1_CC1S_0 << (channel*8U));

		_tim -> CCMR1 |= ((conf -> filter << TIM_CCMR1_IC1F_Pos) << (channel*8U));

		_tim -> CCMR1 |= (conf -> prescaler << (channel*8U));
	}
	else if(channel >= TIM_CHANNEL3 && channel < TIM_NOCHANNEL){
		_tim -> CCMR2 &=~ (0xFF << ((channel-2)));
		_tim -> CCMR2 |= (TIM_CCMR2_CC3S_0 << ((channel-2)*8U));

		_tim -> CCMR2 |= ((conf -> filter << TIM_CCMR2_IC3F_Pos) << ((channel-2)*8U));

		_tim -> CCMR2 |=(conf -> prescaler << (channel*8U));
	}
	_tim -> CCER &=~ ((TIM_CCER_CC1P | TIM_CCER_CC1NP) << (channel*4));
	_tim -> CCER |= ((conf -> polarity << TIM_CCER_CC1P_Pos) << (channel*4));

	return ret;
}

return_t TIM::inputcapture_start(tim_channel_t channel){
	return_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, BUSY, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::inputcapture_stop(tim_channel_t channel){
	return_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, ERR, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}

return_t TIM::inputcapture_start_it(tim_channel_t channel){
	return_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, BUSY, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> DIER |= (TIM_DIER_CC1IE << channel);
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::inputcapture_stop_it(tim_channel_t channel){
	return_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, ERR, __LINE__);
		STM_LOGW(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> DIER &=~ (TIM_DIER_CC1IE << channel);
	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_DisableIRQ(IRQn);

	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}

#ifdef ENABLE_DMA
return_t TIM::inputcapture_start_dma(tim_channel_t channel, uint32_t *capture_buffer, uint16_t size){
	return_t ret;
	dma_t dma = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dma = _conf -> dma_ch1;
		break;
		case TIM_CHANNEL2:
			dma = _conf -> dma_ch2;
		break;
		case TIM_CHANNEL3:
			dma = _conf -> dma_ch3;
		break;
		case TIM_CHANNEL4:
			dma = _conf -> dma_ch4;
		break;
		default:
		break;
	};

	if(dma == NULL){
		set_return(&ret, NOTAVAILABLE, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer not set dma, can't start in dma mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_tim -> DIER &=~ TIM_DIER_CC1DE << channel;
	_tim -> CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	switch(channel){
		case TIM_CHANNEL1:
			CCRx_addr = (uint32_t)&_tim -> CCR1;
		break;
		case TIM_CHANNEL2:
			CCRx_addr = (uint32_t)&_tim -> CCR2;
		break;
		case TIM_CHANNEL3:
			CCRx_addr = (uint32_t)&_tim -> CCR3;
		break;
		case TIM_CHANNEL4:
			CCRx_addr = (uint32_t)&_tim -> CCR4;
		break;
		default:
		break;
	};
	ret = dma -> start(CCRx_addr, (uint32_t)capture_buffer, size);
	if(!is_oke(&ret)) {
		set_return_line(&ret, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
		return ret;
	}
	_tim -> DIER |= TIM_DIER_CC1DE << channel;
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::inputcapture_stop_dma(tim_channel_t channel){
	return_t ret;
	dma_t dma = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dma = _conf -> dma_ch1;
		break;
		case TIM_CHANNEL2:
			dma = _conf -> dma_ch2;
		break;
		case TIM_CHANNEL3:
			dma = _conf -> dma_ch3;
		break;
		case TIM_CHANNEL4:
			dma = _conf -> dma_ch4;
		break;
		default:
		break;
	};

	if(dma == NULL){
		set_return(&ret, NOTAVAILABLE, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer not set dma, can't stop in dma mode.", __FILE__, __FUNCTION__);
		return ret;
	}

	if((_tim -> DIER & TIM_DIER_CC1DE << channel) && (_tim -> CR1 & TIM_CR1_CEN)){
		_tim -> DIER &=~ (TIM_DIER_CC1DE << channel);
		ret = dma -> stop();
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
			STM_LOGE(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
			return ret;
		}
		_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, Timer dma not started.", __FILE__, __FUNCTION__);
	}

	return ret;
}
#endif

uint32_t TIM::get_capture_counter(tim_channel_t channel){
	switch(channel){
		case TIM_CHANNEL1:
			return _tim -> CCR1;

		case TIM_CHANNEL2:
			return _tim -> CCR2;

		case TIM_CHANNEL3:
			return _tim -> CCR3;

		case TIM_CHANNEL4:
			return _tim -> CCR4;
		default:
			return 0U;
	};
	return 0U;
}

/* TIMER OuTPUT COMPARE MODE*/
return_t TIM::set_mode_outputcompare(tim_channel_t channel, tim_outputcompare_t *conf){
	return_t ret;

	if(conf -> port != NULL){
		gpio_alternatefunction_t func;
		if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
		else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
		else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
		else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
		else {
			set_return(&ret, UNSUPPORTED, __LINE__);
			return ret;
		}

		gpio_port_clock_enable(conf -> port);
		gpio_set_alternatefunction(conf -> port, conf -> pin, func);
		gpio_set_alternatefunction_type(conf -> port, conf -> pin, GPIO_OUTPUT_PUSHPULL);
	}

	if	   (_tim == TIM1)  IRQn = TIM1_CC_IRQn;
	else if(_tim == TIM8)  IRQn = TIM8_CC_IRQn;

	_tim -> CCER &=~ TIM_CCER_CC1E;

	if(channel < TIM_CHANNEL3){
		_tim -> CCMR1 &=~ (0xFF << (channel*8));
		_tim -> CCMR1 |= ((conf -> mode << TIM_CCMR1_OC1M_Pos) << (channel*8));
	}
	else if (channel > TIM_CHANNEL2 && channel < TIM_NOCHANNEL){
		_tim -> CCMR2 &=~ (0xFF << ((channel-2)*8));
		_tim -> CCMR2 |= ((conf -> mode << TIM_CCMR2_OC3M_Pos) << ((channel-2)*8));
	}
	_tim -> CCER &=~ (0x0F << (channel*4));
	_tim -> CCER |= ((conf -> level_polarity << TIM_CCER_CC1P_Pos) << (channel*4));

	return ret;
}

return_t TIM::outputcompare_start(tim_channel_t channel, uint32_t value){
	return pwm_output_start(channel, value);
}

return_t TIM::outputcompare_stop(tim_channel_t channel){
	return pwm_output_stop(channel);
}

return_t TIM::outputcompare_start_it(tim_channel_t channel, uint32_t value){
	return pwm_output_start_it(channel, value);
}

return_t TIM::outputcompare_stop_it(tim_channel_t channel){
	return pwm_output_stop_it(channel);
}

#ifdef ENABLE_DMA
return_t TIM::outputcompare_start_dma(tim_channel_t channel, uint32_t *oc_buffer, uint16_t size){
	return pwm_output_start_dma(channel, oc_buffer, size);
}

return_t TIM::outputcompare_stop_dma(tim_channel_t channel){
	return pwm_output_stop_dma(channel);
}
#endif


return_t TIM::set_pulse(tim_channel_t channel, uint32_t pulse){
	return_t ret;


	return ret;
}


void TIM_IRQHandler(tim_t timptr){
	tim_event_t event = TIM_NO_EVENT;
	tim_channel_t channel = TIM_NOCHANNEL;

	timptr -> counter = timptr -> _tim -> CNT;

	/* TIMER CAPTURE-COMPARE 1 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC1IF && timptr -> _tim -> DIER & TIM_DIER_CC1IE){
		timptr -> _tim -> SR =~ TIM_SR_CC1IF;
		event = TIM_CAPTURECOMPARE1_EVENT;
		channel = TIM_CHANNEL1;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 2 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC2IF && timptr -> _tim -> DIER & TIM_DIER_CC2IE){
		timptr -> _tim -> SR =~ TIM_SR_CC2IF;
		event = TIM_CAPTURECOMPARE2_EVENT;
		channel = TIM_CHANNEL2;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 3 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC3IF && timptr -> _tim -> DIER & TIM_DIER_CC3IE){
		timptr -> _tim -> SR =~ TIM_SR_CC3IF;
		event = TIM_CAPTURECOMPARE3_EVENT;
		channel = TIM_CHANNEL3;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 4 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC4IF && timptr -> _tim -> DIER & TIM_DIER_CC4IE){
		timptr -> _tim -> SR =~ TIM_SR_CC4IF;
		event = TIM_CAPTURECOMPARE4_EVENT;
		channel = TIM_CHANNEL4;
		goto EventCB;
	}

	/* TIMER UPDATE INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_UIF && timptr -> _tim -> DIER & TIM_DIER_UIE){
		timptr -> _tim -> SR =~ TIM_SR_UIF;
		event = TIM_UPDATE_EVENT;
		goto EventCB;
	}

	/* TIMER BREAK INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_BIF && timptr -> _tim -> DIER & TIM_DIER_BIE){
		timptr -> _tim -> SR =~ TIM_SR_BIF;
		event = TIM_BREAK_EVENT;
		goto EventCB;
	}

	/* TIMER TRIGER INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_TIF && timptr -> _tim -> DIER & TIM_DIER_TIE){
		timptr -> _tim -> SR =~ TIM_SR_TIF;
		event = TIM_TRIGER_EVENT;
		goto EventCB;
	}

	EventCB:
	if(timptr -> handler_callback != NULL) timptr -> handler_callback(channel, event, timptr -> parameter);
}

#ifdef ENABLE_TIM1
TIM tim_1(TIM1);
tim_t tim1 = &tim_1;
#ifndef ENABLE_TIM9
void TIM1_BRK_TIM9_IRQHandler(void){
	TIM_IRQHandler(tim1);
}
#endif
#ifndef ENABLE_TIM10
void TIM1_UP_TIM10_IRQHandler(void){
	TIM_IRQHandler(tim1);
}
#endif
#ifndef ENABLE_TIM11
void TIM1_TRG_COM_TIM11_IRQHandler(void){
	TIM_IRQHandler(tim1);
}
#endif
void TIM1_CC_IRQHandler(void){
	TIM_IRQHandler(tim1);
}
#endif

#ifdef ENABLE_TIM2
TIM tim_2(TIM2);
tim_t tim2 = &tim_2;
void TIM2_IRQHandler(void){
	TIM_IRQHandler(tim2);
}
#endif

#ifdef ENABLE_TIM3
TIM tim_3(TIM3);
tim_t tim3 = &tim_3;
void TIM3_IRQHandler(void){
	TIM_IRQHandler(tim3);
}
#endif

#ifdef ENABLE_TIM4
TIM tim_4(TIM4);
tim_t tim4 = &tim_4;
void TIM4_IRQHandler(void){
	TIM_IRQHandler(tim4);
}
#endif

#ifdef ENABLE_TIM5
TIM tim_5(TIM5);
tim_t tim5 = &tim_5;
void TIM5_IRQHandler(void){
	TIM_IRQHandler(tim5);
}
#endif

#ifdef ENABLE_TIM6
TIM tim_6(TIM6);
tim_t tim6 = &tim_6;
void TIM6_DAC_IRQHandler(void){
	TIM_IRQHandler(tim6);
}
#endif

#ifdef ENABLE_TIM7
TIM tim_7(TIM7);
tim_t tim7 = &tim_7;
void TIM7_IRQHandler(void){
	TIM_IRQHandler(tim7);
}
#endif

#ifdef ENABLE_TIM8
TIM tim_8(TIM8);
tim_t tim8 = &tim_8;
#ifndef ENABLE_TIM12
void TIM8_BRK_TIM12_IRQHandler(void){
	TIM_IRQHandler(tim8);
}
#endif
#ifndef ENABLE_TIM13
void TIM8_UP_TIM13_IRQHandler(void){
	TIM_IRQHandler(tim8);
}
#endif
#ifndef ENABLE_TIM14
void TIM8_TRG_COM_TIM14_IRQHandler(void){
	TIM_IRQHandler(tim8);
}
#endif
void TIM8_CC_IRQHandler(void){
	TIM_IRQHandler(tim8);
}
#endif

#ifdef ENABLE_TIM9
TIM tim_9(TIM9);
tim_t tim9 = &tim_9;
void TIM1_BRK_TIM9_IRQHandler(void){
#ifdef ENABLE_TIM1
	TIM_IRQHandler(tim1);
#endif
	TIM_IRQHandler(tim9);
}
#endif

#ifdef ENABLE_TIM10
TIM tim_10(TIM10);
tim_t tim10 = &tim_10;
void TIM1_UP_TIM10_IRQHandler(void){
#ifdef ENABLE_TIM1
	TIM_IRQHandler(tim1);
#endif
	TIM_IRQHandler(tim10);
}
#endif

#ifdef ENABLE_TIM11
TIM tim_11(TIM11);
tim_t tim11 = &tim_11;
void TIM1_TRG_COM_TIM11_IRQHandler(void){
#ifdef ENABLE_TIM1
	TIM_IRQHandler(tim1);
#endif
	TIM_IRQHandler(tim11);
}
#endif

#ifdef ENABLE_TIM12
TIM tim_12(TIM12);
tim_t tim12 = &tim_12;
void TIM8_BRK_TIM12_IRQHandler(void){
#ifdef ENABLE_TIM8
	TIM_IRQHandler(tim8);
#endif
	TIM_IRQHandler(tim12);
}
#endif

#ifdef ENABLE_TIM13
TIM tim_13(TIM13);
tim_t tim13 = &tim_13;
void TIM8_UP_TIM13_IRQHandler(void){
#ifdef ENABLE_TIM8
	TIM_IRQHandler(tim8);
#endif
	TIM_IRQHandler(tim13);
}
#endif

#ifdef ENABLE_TIM14
TIM tim_14(TIM14);
tim_t tim14 = &tim_14;
void TIM8_TRG_COM_TIM14_IRQHandler(void){
#ifdef ENABLE_TIM8
	TIM_IRQHandler(tim8);
#endif
	TIM_IRQHandler(tim14);
}
#endif


#endif


