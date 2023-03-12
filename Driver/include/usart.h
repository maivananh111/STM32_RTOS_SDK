/*
 * usart.h
 *
 *  Created on: 29 thg 7, 2022
 *      Author: A315-56
 */

#ifndef USART_F4XX_H_
#define USART_F4XX_H_


#include "periph_en.h"
#ifdef ENABLE_USART

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "stdio.h"
#include "stm32f4xx.h"
#ifdef ENABLE_DMA
#include "dma.h"
#endif
#include "status.h"


typedef enum{
	USART_NORMAL_CONTROL = 0,
	USART_INTERRUPT_CONTROl,
	USART_DMA_CONTROl,
	USART_INTERRUPT_DMA_CONTROL,
} usart_periph_control_t;

typedef enum{
	USART_RECEIVE_INTERRUPT = 0,
	USART_TRANSMIT_INTERRUPT,
	USART_TRANSMIT_RECEIVE_INTERRUPT,
} usart_interruptselect_t;

typedef enum{
	USART_EVENT_NOEVENT,
	USART_EVENT_TRANSMIT_COMPLETE,
	USART_EVENT_RECEIVE_COMPLETE,
	USART_EVENT_BUFFER_OVERFLOW,
	USART_EVENT_IDLE_STATE,
	USART_EVENT_RECEIVE_ENDCHAR,
	USART_EVENT_ERROR,
} usart_event_t;

typedef enum{
	USART_RECEPTION_NORMAL,
	USART_RECEPTION_TOENDCHAR,
	USART_RECEPTION_TOIDLE,
} usart_reception_t;

typedef struct{
	uint32_t 			     baudrate;
	usart_periph_control_t   control = USART_NORMAL_CONTROL;
	usart_interruptselect_t  interruptselect = USART_RECEIVE_INTERRUPT;
	uint32_t 			     interruptpriority = 0;
	GPIO_TypeDef 		     *txport;
	uint16_t 			     txpin;
	GPIO_TypeDef 		     *rxport;
	uint16_t 			     rxpin;
	dma_t 				     txdma = NULL;
	dma_t 				     rxdma = NULL;
} usart_config_t;

#define USART_TIMEOUT 100U

class USART {
	public:
		USART(USART_TypeDef *usart);
		return_t init(usart_config_t *conf);

		return_t register_event_handler(void (*function_ptr)(usart_event_t event, void *param), void *param = NULL);

		return_t transmit(uint8_t data);
		return_t transmit(uint8_t *data, uint16_t len);
		return_t send_string(char *string);

		return_t receive(uint16_t len);
		return_t receive_to_endchar(char endchar);
		return_t receive(uint8_t *data);
		uint8_t  get_data(void);

		return_t transmit_start_it(uint8_t *data, uint16_t len);
		return_t receive_start_it(uint16_t buffer_size);
		return_t transmit_stop_it(void);
		return_t receive_stop_it(void);

		return_t transmit_start_dma(uint8_t *data, uint16_t len);
		return_t receive_start_dma(uint16_t len);
		return_t transmit_stop_dma(void);
		return_t receive_stop_dma(void);

		return_t receive_to_idle_start_it(uint16_t buffer_size);
		return_t receive_to_idle_stop_it(void);
		return_t receive_to_idle_start_it_dma(uint16_t buffer_size);
		return_t receive_to_idle_stop_it_dma(void);

		return_t receice_to_endchar_start_it(uint16_t buffer_size, char endchar = '\0');
		return_t receice_to_endchar_stop_it(void);
		return_t receice_to_endchar_start_dma(uint16_t buffer_size, char endchar = '\0');
		return_t receice_to_endchar_stop_dma(void);

		return_t get_buffer(uint8_t **data);
		uint16_t get_bufferlen(void);

		usart_config_t *get_config(void);

		USART_TypeDef *_usart;
		dma_t _txdma = NULL, _rxdma = NULL;
		void *parameter = NULL;
		void (*handler_callback)(usart_event_t event, void *param) = NULL;
		uint8_t *rxbuffer = NULL;
		uint16_t rxlen, rxcount;
		char endchar = '\0';
		usart_reception_t reception = USART_RECEPTION_NORMAL;

	private:
		usart_config_t *_conf = NULL;
		IRQn_Type IRQn;

};

void USART_IRQ_Handler(USART *usart);

#if defined(ENABLE_USART1) && defined(USART1)
extern USART usart1;
void USART1_IRQHandler(void);
#endif
#if defined(ENABLE_USART2) && defined(USART2)
extern USART usart2;
void USART2_IRQHandler(void);
#endif
#if defined(ENABLE_USART3) && defined(USART3)
extern USART usart3;
void USART3_IRQHandler(void);
#endif
#if defined(ENABLE_UART4) && defined(UART4)
extern USART uart4;
void UART4_IRQHandler(void);
#endif
#if defined(ENABLE_UART5) && defined(UART5)
extern USART uart5;
void UART5_IRQHandler(void);
#endif
#if defined(ENABLE_USART6) && defined(USART6)
extern USART usart6;
void USART6_IRQHandler(void);
#endif
#if defined(ENABLE_USART7) && defined(USART7)
extern USART usart7;
void USART7_IRQHandler(void);
#endif


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_USART */

#endif /* USART_F4XX_H_ */

