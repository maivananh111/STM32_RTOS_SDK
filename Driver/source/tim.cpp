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
	_tim -> CR1 |= (_conf -> direction << TIM_CR1_DIR_Pos) | (_conf -> autoreloadpreload << TIM_CR1_ARPE_Pos);

	_tim -> ARR = _conf -> reload - 1;
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

return_t TIM::set_mode_pwm(tim_channel_t channel, tim_pwmmode_t mode, GPIO_TypeDef *port, uint16_t pin){
	return_t ret;

	gpio_port_clock_enable(port);

	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
	else {
		set_return(&ret, UNSUPPORTED, __LINE__);
		return ret;
	}
	gpio_set_alternatefunction(port, pin, func);

	if(channel < TIM_CHANNEL3){ // Channel 1-2
		_tim -> CCMR1 |=  (TIM_CCMR1_OC1PE << (channel*8)); // Set PE.
		_tim -> CCMR1 &=~ (TIM_CCMR1_OC1FE << (channel*8)); // Clear FE.
		_tim -> CCMR1 |=  ((mode << TIM_CCMR1_OC1M_Pos) << (channel*8)); // Set 6UL to OCxM.
	}
	else if(channel >= TIM_CHANNEL3 && channel < TIM_NOCHANNEL){ // Channel 3-4
		_tim -> CCMR2 |=  (TIM_CCMR2_OC3PE << ((channel - 2)*8)); // Set PE.
		_tim -> CCMR2 &=~ (TIM_CCMR2_OC3FE << ((channel - 2)*8)); // Clear FE.
		_tim -> CCMR2 |=  ((mode << TIM_CCMR2_OC3M_Pos) << ((channel - 2)*8)); // Set PWM mode1 or mode2 (mode1 notinvert: 6, mode2 invert: 7) to OCxM.
	}

	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));

	return ret;
}

return_t TIM::set_mode_encoder(GPIO_TypeDef *a_port, uint16_t a_pin, GPIO_TypeDef *b_port, uint16_t b_pin){
	return_t ret;
/*
	if	   (_tim == TIM1)  IRQn = TIM1_CC_IRQn;
	else if(_tim == TIM8)  IRQn = TIM8_CC_IRQn;

		gpio_port_clock_enable(_conf -> enc1port);
		gpio_port_clock_enable(_conf -> enc2port);

		gpio_set_mode(_conf -> enc1port, _conf -> enc1pin, GPIO_INPUT_PULLUP);
		gpio_set_mode(_conf -> enc2port, _conf -> enc2pin, GPIO_INPUT_PULLUP);

		_tim -> ARR = 0xFFFE;
		_tim -> PSC = 0;

		_tim -> SMCR |= (3UL << TIM_SMCR_SMS_Pos);
		_tim -> CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
		_tim -> CCMR1 &=~ (TIM_CCMR1_IC1PSC | TIM_CCMR1_IC2PSC);
		_tim -> CCMR1 &=~ (TIM_CCMR1_IC1F | TIM_CCMR1_IC2F);

		_tim -> CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P;
		_tim -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

	}
	*/
	return ret;
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


return_t TIM::register_event_handler(void(*function_ptr)(tim_event_t event, void *param), void *param){
	return_t ret;

	if(_conf -> interrupt != TIM_INTERRUPT_ENABLE) {
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disable, can't register timer event handler.", __FILE__, __FUNCTION__);
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
	_tim -> CR1 &=! TIM_CR1_CEN;

	return ret;
}

return_t TIM::start_it(void){
	return_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, UNAVAILABLE, __LINE__);
		STM_LOGE(TAG, "%s -> %s -> Timer interrupt disable.", __FILE__, __FUNCTION__);
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
		STM_LOGW(TAG, "%s -> %s, Timer started, can't restart in interrut mode.", __FILE__, __FUNCTION__);
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
		set_return(&ret, UNAVAILABLE, __LINE__);
		return ret;
	}

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, ERR, __LINE__);
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

return_t TIM::start_dma(uint32_t *count_buf, uint16_t size){
	return_t ret;

	_tim -> CR1  &=~ TIM_CR1_CEN;

	ret = _conf -> dma -> start((uint32_t)count_buf, (uint32_t)&_tim -> CNT, size);
	if(!is_oke(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}

	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::stop_dma(void){
	return_t ret;

	if(_tim -> DIER & TIM_DIER_CC1DE){
		ret = _conf -> dma -> stop();
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
			return ret;
		}

		_tim -> DIER &=~ TIM_DIER_CC1DE;
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, ERR, __LINE__);
	}

	return ret;
}


/* TIMER PWM MODE */
return_t TIM::pwm_set_duty(tim_channel_t channel, uint16_t pwm){
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


return_t TIM::pwm_start_dma(tim_channel_t channel, uint16_t *pwm, uint16_t size){
	return_t ret;

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
	ret = _conf -> dma -> start((uint32_t)pwm, CCRx_addr, size);
	if(!is_oke(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}
	_tim -> DIER |= TIM_DIER_CC1DE << channel;

	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

return_t TIM::pwm_stop_dma(tim_channel_t channel){
	return_t ret;

	if(_tim -> DIER & TIM_DIER_CC1DE <<channel){
		_tim -> DIER &=~ TIM_DIER_CC1DE << channel;
		ret = _conf -> dma -> stop();
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, ERR, __LINE__);
	}

	return ret;
}

/* TIMER ENCODER MODE */
/*
void TIM::Encoder_ISR(void){
	if(_tim -> SR & TIM_SR_CC1IF){
		_tim -> SR =~ TIM_SR_CC1IF;
		counter = _tim -> CNT;
	}
	if(_tim -> SR & TIM_SR_CC2IF){
		_tim -> SR =~ TIM_SR_CC2IF;
		counter = _tim -> CNT;
	}
}

int16_t TIM::Encoder_GetValue(void){
	return (int16_t)((int16_t)counter/4);
}
*/

void TIM_IRQHandler(tim_t timptr){
	tim_event_t event = TIM_NO_EVENT;

	timptr -> counter = timptr -> _tim -> CNT;

	/* TIMER CAPTURE-COMPARE 1 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC1IF && timptr -> _tim -> DIER & TIM_DIER_CC1DE){
		timptr -> _tim -> SR =~ TIM_SR_CC1IF;
		event = TIM_CAPTURECOMPARE1_EVENT;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 2 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC2IF && timptr -> _tim -> DIER & TIM_DIER_CC2DE){
		timptr -> _tim -> SR =~ TIM_SR_CC2IF;
		event = TIM_CAPTURECOMPARE2_EVENT;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 3 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC3IF && timptr -> _tim -> DIER & TIM_DIER_CC3DE){
		timptr -> _tim -> SR =~ TIM_SR_CC3IF;
		event = TIM_CAPTURECOMPARE3_EVENT;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 4 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC4IF && timptr -> _tim -> DIER & TIM_DIER_CC4DE){
		timptr -> _tim -> SR =~ TIM_SR_CC4IF;
		event = TIM_CAPTURECOMPARE4_EVENT;
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
	timptr -> handler_callback(event, timptr -> parameter);
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


