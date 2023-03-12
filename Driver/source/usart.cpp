/*
 * usart.cpp
 *
 *  Created on: 20 thg 11, 2022
 *      Author: anh
 */
#include "periph_en.h"
#ifdef ENABLE_USART
#include "usart.h"

#include "math.h"
#include "string.h"

#include "gpio.h"
#include "systick.h"
#include "rcc.h"

#include "stm_log.h"



static const char *TAG = "USART";

USART::USART(USART_TypeDef *usart){
	_usart = usart;
}

return_t USART::init(usart_config_t *conf){
	return_t ret;
	_conf = conf;
	_rxdma = _conf -> rxdma;
	_txdma = _conf -> txdma;
	__IO uint32_t usart_bus_frequency = 0UL;

	gpio_port_clock_enable(_conf -> txport);
	gpio_port_clock_enable(_conf -> rxport);
	if(
#if defined(USART1)
			_usart == USART1
#endif /* defined(USART1) */
			||
#if defined(USART2)
			_usart == USART2
#endif /* defined(USART2) */
			||
#if defined(USART3)
			_usart == USART3
#endif /* defined(USART3) */
			){
		gpio_set_alternatefunction(_conf -> txport, _conf -> txpin, AF7_USART1_3);
		gpio_set_alternatefunction(_conf -> rxport, _conf -> rxpin, AF7_USART1_3);
	}
	else{
		gpio_set_alternatefunction(_conf -> txport, _conf -> txpin, AF8_USART4_6);
		gpio_set_alternatefunction(_conf -> rxport, _conf -> rxpin, AF8_USART4_6);
	}
	gpio_set_alternatefunction_type(_conf -> txport, _conf -> txpin, GPIO_OUTPUT_PUSHPULL);
	gpio_set_alternatefunction_type(_conf -> rxport, _conf -> rxpin, GPIO_OUTPUT_PUSHPULL);

#if defined(USART1)
	if(_usart == USART1){
		RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB2);
	}
#endif /* defined(USART1) */
#if defined(USART2)
	if(_usart == USART2){
		RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(USART2) */
#if defined(USART3)
	if(_usart == USART3){
		RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(USART3) */
#if defined(UART4)
	if(_usart == UART4){
		RCC -> APB1ENR |= RCC_APB1ENR_UART4EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(UART4) */
#if defined(UART5)
	if(_usart == UART5){
		RCC -> APB1ENR |= RCC_APB1ENR_UART5EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(UART5) */
#if defined(USART6)
	if(_usart == USART6){
		RCC -> APB2ENR |= RCC_APB2ENR_USART6EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB2);
	}
#endif /* defined(USART6) */


	_usart -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

	float USARTDIV = (float)(usart_bus_frequency/(_conf -> baudrate * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	DIV_Fraction = ceil(Fraction);
	_usart -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	if(_conf -> control == USART_INTERRUPT_CONTROl || _conf -> control == USART_INTERRUPT_DMA_CONTROL){

		if(_conf -> interruptpriority < RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
			set_return(&ret, ERR, __LINE__);
			STM_LOGE(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
			return ret;
		}

#if defined(USART1)
		if	   (_usart == USART1) IRQn = USART1_IRQn;
#endif /* defined(USART1) */
#if defined(USART2)
		else if(_usart == USART2) IRQn = USART2_IRQn;
#endif /* defined(USART2) */
#if defined(USART3)
		else if(_usart == USART3) IRQn = USART3_IRQn;
#endif /* defined(USART3) */
#if defined(UART4)
		else if(_usart == UART4)  IRQn = UART4_IRQn;
#endif /* defined(USART4) */
#if defined(UART5)
		else if(_usart == UART5)  IRQn = UART5_IRQn;
#endif /* defined(USART5) */
#if defined(USART6)
		else if(_usart == USART6) IRQn = USART6_IRQn;
#endif /* defined(USART6) */
	}
	transmit('\n');

	return {OKE, 0};
}

return_t USART::register_event_handler(void (*function_ptr)(usart_event_t event, void *param), void *param){
	return_t ret;
	if(_conf -> control == USART_INTERRUPT_CONTROl || _conf -> control == USART_INTERRUPT_DMA_CONTROL) {
		handler_callback = function_ptr;
		parameter = param;

		return ret;
	}
	else{
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGW(TAG, "%s -> %s, USART peripheral control unsuported register event handler.", __FILE__, __FUNCTION__ );
	}

	return ret;


}


return_t USART::transmit(uint8_t data){
	return_t ret;

	_usart -> DR = data;

	ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return ret;
}



return_t USART::transmit(uint8_t *data, uint16_t len){
	return_t ret;
	uint16_t TxCount = len;

	while(TxCount--) {
		_usart -> DR = *data++;

		ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	return ret;
}



return_t USART::send_string(char *string){
	return_t ret;

	while(*string) {
		_usart -> DR = *string++;

		ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	return ret;
}



return_t USART::receive(uint8_t *data){
	return_t ret;

	ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_RXNE, FLAG_SET, USART_TIMEOUT);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	(*data) = _usart -> DR;

	return ret;
}



return_t USART::receive(uint16_t len){
	return_t ret;
	__IO uint16_t RxCount = len;

	rxlen = len;
	rxcount = 0;
	reception = USART_RECEPTION_NORMAL;
	if(rxbuffer != NULL) {
		free(rxbuffer);
	}
	rxbuffer = (uint8_t *)malloc(rxlen * sizeof(uint8_t));

	while(RxCount--){
		ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_RXNE, FLAG_SET, USART_TIMEOUT);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}

		(*rxbuffer++) = _usart -> DR;
	}
	return ret;
}

uint8_t USART::get_data(void){
	return _usart -> DR;
}


return_t USART::transmit_start_it(uint8_t *data, uint16_t len){
	return_t ret;

	if(_conf -> control == USART_INTERRUPT_CONTROl || _conf -> control == USART_INTERRUPT_DMA_CONTROL) {
		if(_conf -> interruptselect == USART_TRANSMIT_INTERRUPT || _conf -> interruptselect == USART_TRANSMIT_RECEIVE_INTERRUPT)
			_usart -> CR1 |= USART_CR1_TCIE;
		else
			STM_LOGE(TAG, "%s -> %s, USART not selected transmit interrupt.", __FILE__, __FUNCTION__ );
	}
	else{
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s, USART not selected interrupt control.", __FILE__, __FUNCTION__ );
		return ret;
	}

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;

	uint16_t TxLength = len;
	uint16_t TxCount  = 0;
	while(TxCount++ < TxLength) {
		_usart -> DR = *data++;

		ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}

	_usart -> CR1 &=~ (USART_CR1_PEIE | USART_CR1_TCIE);
	_usart -> CR3 &=~ USART_CR3_EIE;

	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_EnableIRQ(IRQn);

	tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return ret;
}



return_t USART::receive_start_it(uint16_t buffer_size){
	return_t ret;

	if(_conf -> control == USART_INTERRUPT_CONTROl || _conf -> control == USART_INTERRUPT_DMA_CONTROL) {
		if(_conf -> interruptselect == USART_RECEIVE_INTERRUPT || _conf -> interruptselect == USART_TRANSMIT_RECEIVE_INTERRUPT)
			_usart -> CR1 |= USART_CR1_RXNEIE;
		else
			STM_LOGE(TAG, "%s -> %s, USART not selected receive interrupt.", __FILE__, __FUNCTION__ );
	}
	else{
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s, USART not selected interrupt control.", __FILE__, __FUNCTION__ );
		return ret;
	}

	rxlen = buffer_size;
	rxcount = 0;
	reception = USART_RECEPTION_NORMAL;
	if(rxbuffer != NULL) {
		free(rxbuffer);
	}
	rxbuffer = (uint8_t *)malloc(rxlen * sizeof(uint8_t));

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;

	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_EnableIRQ(IRQn);

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return ret;
}

return_t USART::transmit_stop_it(void){
	return_t ret;

	if(_conf -> control == USART_INTERRUPT_CONTROl || _conf -> control == USART_INTERRUPT_DMA_CONTROL) {
		if(((_conf -> interruptselect == USART_TRANSMIT_INTERRUPT) || (_conf -> interruptselect == USART_TRANSMIT_RECEIVE_INTERRUPT))\
				&& (_usart -> CR1 & USART_CR1_TCIE)){

			_usart -> CR1 &=~ USART_CR1_TCIE;

			__NVIC_ClearPendingIRQ(IRQn);
			__NVIC_DisableIRQ(IRQn);

			return ret;
		}
		else
			STM_LOGE(TAG, "%s -> %s, USART not selected transmit interrupt.", __FILE__, __FUNCTION__ );
	}

	set_return(&ret, ERR, __LINE__);
	STM_LOGE(TAG, "%s -> %s, USART not started transmit interrupt.", __FILE__, __FUNCTION__ );

	return ret;
}

return_t USART::receive_stop_it(void){
	return_t ret;

	if(_conf -> control == USART_INTERRUPT_CONTROl || _conf -> control == USART_INTERRUPT_DMA_CONTROL) {
		if(((_conf -> interruptselect == USART_RECEIVE_INTERRUPT) || (_conf -> interruptselect == USART_TRANSMIT_RECEIVE_INTERRUPT))\
					&& (_usart -> CR1 & USART_CR1_RXNEIE)){

			_usart -> CR1 &=~ USART_CR1_RXNEIE;

			__NVIC_ClearPendingIRQ(IRQn);
			__NVIC_DisableIRQ(IRQn);

			if(rxbuffer != NULL) free(rxbuffer);
			rxcount = 0;
			rxlen = 0;
			reception = USART_RECEPTION_NORMAL;

			return ret;
		}
		else
			STM_LOGE(TAG, "%s -> %s, USART not selected receive interrupt.", __FILE__, __FUNCTION__ );
	}

	set_return(&ret, ERR, __LINE__);
	STM_LOGE(TAG, "%s -> %s, USART not started receive interrupt.", __FILE__, __FUNCTION__ );

	return ret;
}

return_t USART::transmit_start_dma(uint8_t *data, uint16_t len){
	return_t ret;

	if(_conf -> control != USART_DMA_CONTROl || _conf -> control != USART_INTERRUPT_DMA_CONTROL || _txdma == NULL){
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s, USART not selected dma control.", __FILE__, __FUNCTION__ );
		return ret;
	}

	_usart -> CR3 &=~ USART_CR3_DMAT;
	ret = _txdma -> start((uint32_t)data, (uint32_t)&_usart -> DR, len);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	_usart -> SR &=~ USART_SR_TC;
	_usart -> CR3 |= USART_CR3_DMAT;

	return ret;
}

return_t USART::receive_start_dma(uint16_t len){
	return_t ret;

	if(_conf -> control != USART_DMA_CONTROl || _conf -> control != USART_INTERRUPT_DMA_CONTROL || _rxdma == NULL){
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s, USART not selected dma control.", __FILE__, __FUNCTION__ );
		return ret;
	}

	rxlen = len;
	rxcount = 0;
	reception = USART_RECEPTION_NORMAL;
	if(rxbuffer != NULL) {
		free(rxbuffer);
	}
	rxbuffer = (uint8_t *)malloc(rxlen * sizeof(uint8_t));

	_usart -> CR3 &=~ USART_CR3_DMAR;
	ret = _rxdma -> start((uint32_t)&_usart -> DR, (uint32_t)rxbuffer, len);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;
	_usart -> CR3 |= USART_CR3_DMAR;

	return ret;
}

return_t USART::transmit_stop_dma(void){
	return_t ret;

	if(_conf -> control != USART_DMA_CONTROl || _conf -> control != USART_INTERRUPT_DMA_CONTROL || _txdma == NULL){
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s, USART not selected dma control.", __FILE__, __FUNCTION__ );
		return ret;
	}

	if(_usart -> CR3 & USART_CR3_DMAT){
		_usart -> CR3 &=~ USART_CR3_DMAT;
		ret = _txdma -> stop();
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_usart -> CR1 &=~ USART_CR1_TXEIE;
	}

	else{
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, USART not started transmit dma.", __FILE__, __FUNCTION__ );
	}

	return ret;
}

return_t USART::receive_stop_dma(void){
	return_t ret;

	if(_conf -> control != USART_DMA_CONTROl || _conf -> control != USART_INTERRUPT_DMA_CONTROL || _rxdma == NULL){
		set_return(&ret, UNSUPPORTED, __LINE__);
		STM_LOGE(TAG, "%s -> %s, USART not selected dma control.", __FILE__, __FUNCTION__ );
		return ret;
	}

	if(_usart -> CR3 & USART_CR3_DMAR){
		_usart -> CR3 &=~ USART_CR3_DMAR;
		ret = _rxdma -> stop();
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_usart -> CR1 &=~ USART_CR1_PEIE;
		_usart -> CR3 &=~ USART_CR3_EIE;

		if(rxbuffer != NULL) free(rxbuffer);
		rxlen = 0;
		rxcount = 0;
		reception = USART_RECEPTION_NORMAL;
	}

	else{
		set_return(&ret, ERR, __LINE__);
		STM_LOGE(TAG, "%s -> %s, USART not started receive dma.", __FILE__, __FUNCTION__ );
	}

	return ret;
}




return_t USART::receive_to_idle_start_it(uint16_t buffer_size){
	return_t ret = receive_start_it(buffer_size);

	reception = USART_RECEPTION_TOIDLE;

	_usart -> CR1 |= USART_CR1_IDLEIE;

	return ret;
}
return_t USART::receive_to_idle_stop_it(void){
	return_t ret = receive_stop_it();

	_usart -> CR1 &=~ USART_CR1_IDLEIE;

	return ret;
}

return_t USART::receive_to_idle_start_it_dma(uint16_t buffer_size){
	return_t ret = receive_start_dma(buffer_size);

	reception = USART_RECEPTION_TOIDLE;

	_usart -> CR1 |= USART_CR1_IDLEIE;

	return ret;
}

return_t USART::receive_to_idle_stop_it_dma(void){
	return_t ret = receive_stop_dma();

	_usart -> CR1 &=~ USART_CR1_IDLEIE;

	return ret;
}


return_t USART::receice_to_endchar_start_it(uint16_t buffer_size, char endchar){
	return_t ret =receive_start_it(buffer_size);

	this->endchar = endchar;
	reception = USART_RECEPTION_TOENDCHAR;

	return ret;
}

return_t USART::receice_to_endchar_stop_it(void){
	this->endchar = '\0';
	return receive_stop_it();
}

return_t USART::receice_to_endchar_start_dma(uint16_t buffer_size, char endchar){
	return_t ret =receive_start_dma(buffer_size);

	this->endchar = endchar;
	reception = USART_RECEPTION_TOENDCHAR;

	return ret;
}

return_t USART::receice_to_endchar_stop_dma(void){
	this->endchar = '\0';
	return receive_stop_dma();
}

return_t USART::get_buffer(uint8_t **data){
	return_t ret;

	if(rxbuffer != NULL){
		rxbuffer[rxlen] = '\0';
		*data = (uint8_t*)malloc(rxlen+1);
		memcpy(*data, rxbuffer, rxlen+1);

		free(rxbuffer);

		rxcount = 0;
		rxbuffer = (uint8_t *)malloc(rxlen * sizeof(uint8_t));
		return ret;
	}

	set_return(&ret, ERR, __LINE__);
	STM_LOGE(TAG, "%s -> %s, USART receive buffer empty.", __FILE__, __FUNCTION__ );

	return ret;
}

uint16_t USART::get_bufferlen(void){
	return rxcount;
}

usart_config_t *USART::get_config(void){
	return _conf;
}


void USART_IRQ_Handler(USART *usart){
	uint32_t StatusReg = usart -> _usart -> SR, CR1Reg = usart -> _usart -> CR1;
	usart_event_t event = USART_EVENT_NOEVENT;
	if(StatusReg & USART_SR_RXNE && CR1Reg & USART_CR1_RXNEIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
		tmp = usart -> _usart -> DR;
		(void)tmp;

		event = USART_EVENT_RECEIVE_COMPLETE;
		usart -> _usart -> SR &=~ USART_SR_RXNE;

		if(usart -> rxcount < usart -> rxlen)
			usart -> rxbuffer[usart -> rxcount] = usart -> _usart -> DR;
		else{
			event = USART_EVENT_BUFFER_OVERFLOW;
			goto EventCB;
		}
		if(usart -> reception == USART_RECEPTION_TOENDCHAR){
			if(usart -> rxbuffer[usart -> rxcount] == usart -> endchar) {
				event = USART_EVENT_RECEIVE_ENDCHAR;
				usart -> _usart -> CR1 &=~ USART_CR1_PEIE;
				usart -> _usart -> CR3 &=~ USART_CR3_EIE;
			}
		}
		usart -> rxcount++;
		goto EventCB;
	}

	if(StatusReg & USART_SR_TC&& CR1Reg & USART_CR1_TCIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
		tmp = usart -> _usart -> DR;
		(void)tmp;

		event = USART_EVENT_TRANSMIT_COMPLETE;
		usart -> _usart -> SR &=~ USART_SR_TC;

		goto EventCB;
	}

	if(StatusReg & USART_SR_IDLE && CR1Reg & USART_CR1_IDLEIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
		tmp = usart -> _usart -> DR;
		(void)tmp;

		if(usart -> reception == USART_RECEPTION_TOIDLE){
			usart -> _usart -> SR &=~ (USART_SR_IDLE | USART_SR_RXNE);
			if(usart -> _usart -> CR3 & USART_CR3_DMAR){
				usart -> rxcount = usart -> _rxdma -> get_counter();
				if(usart -> _rxdma -> get_config() -> mode != DMA_CIRCULAR){
//					usart -> _usart -> CR3 &=~ (USART_CR3_EIE | USART_CR3_DMAR);
					usart -> _rxdma -> stop();
				}
				goto EventCB;
			}
			else{
				event = USART_EVENT_IDLE_STATE;
				goto EventCB;
			}
		}
	}

	EventCB:
	if(usart -> handler_callback != NULL) usart -> handler_callback(event, usart -> parameter);

}

#if defined(ENABLE_USART1) && defined(USART1)
USART usart1(USART1);
void USART1_IRQHandler(void){
	USART_IRQ_Handler(&usart1);
}
#endif /* defined(ENABLE_USART1) && defined(USART1) */
#if defined(ENABLE_USART2) && defined(USART2)
USART usart2(USART2);
void USART2_IRQHandler(void){
	USART_IRQ_Handler(&usart2);
}
#endif /* defined(ENABLE_USART2) && defined(USART2) */
#if defined(ENABLE_USART3) && defined(USART3)
USART usart3(USART3);
void USART3_IRQHandler(void){
	USART_IRQ_Handler(&usart3);
}
#endif /* defined(ENABLE_USART3) && defined(USART3) */
#if defined(ENABLE_UART4) && defined(UART4)
USART uart4 (UART4);
void UART4_IRQHandler(void){
	USART_IRQ_Handler(&uart4);
}
#endif /* defined(ENABLE_USART1) && defined(USART1) */
#if defined(ENABLE_UART5) && defined(UART5)
USART uart5 (UART5);
void UART5_IRQHandler(void){
	USART_IRQ_Handler(&uart5);
}
#endif /* defined(ENABLE_UART5) && defined(UART5) */
#if defined(ENABLE_USART6) && defined(USART6)
USART usart6(USART6);
void USART6_IRQHandler(void){
	USART_IRQ_Handler(&usart6);
}
#endif /* defined(ENABLE_USART6) && defined(USART6) */


#endif /* ENABLE_USART */





