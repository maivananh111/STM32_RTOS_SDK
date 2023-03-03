/*
 * tim.h
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */

#ifndef TIM_H_
#define TIM_H_

#include "periph_en.h"
#ifdef ENABLE_TIM


#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include "status.h"

#ifdef ENABLE_DMA
#include "dma.h"
#endif


typedef enum{
	TIM_CHANNEL1 = 0,
	TIM_CHANNEL2,
	TIM_CHANNEL3,
	TIM_CHANNEL4,
	TIM_NOCHANNEL,
} tim_channel_t;

typedef enum{
	TIM_COUNTER_UP = 0,
	TIM_COUNTER_DOWN,
} tim_direction_t;

typedef enum{
	TIM_ARP_DISABLE = 0,
	TIM_ARP_ENABLE,
} tim_ARPE_t;

typedef enum{
	TIM_INTERRUPT_DISABLE = 0,
	TIM_INTERRUPT_ENABLE = 1,
} tim_interrupt_t;

typedef enum{
	TIM_PWM_NOINVERT = 6,
	TIM_PWM_INVERT = 7,
} tim_pwmmode_t;

typedef enum{
	TIM_UPDATE_EVENT,
	TIM_BREAK_EVENT,
	TIM_TRIGER_EVENT,
	TIM_CAPTURECOMPARE1_EVENT,
	TIM_CAPTURECOMPARE2_EVENT,
	TIM_CAPTURECOMPARE3_EVENT,
	TIM_CAPTURECOMPARE4_EVENT,
	TIM_NO_EVENT,
}tim_event_t;

typedef struct{
	uint32_t 		prescaler = 0U;
	uint32_t 	    reload = 0U;
	tim_direction_t direction =TIM_COUNTER_UP;
	tim_ARPE_t		autoreloadpreload = TIM_ARP_DISABLE;
	tim_interrupt_t interrupt = TIM_INTERRUPT_DISABLE;
	uint32_t        interruptpriority = 0U;
#ifdef ENABLE_DMA
	dma_t           dma = NULL;
#endif
} tim_config_t;

class TIM{
	public:
		TIM(TIM_TypeDef *Timer);

		return_t init(tim_config_t *conf);
		return_t set_mode_pwm(tim_channel_t channel, tim_pwmmode_t mode, GPIO_TypeDef *port, uint16_t pin);
		return_t set_mode_encoder(GPIO_TypeDef *a_port, uint16_t a_pin, GPIO_TypeDef *b_port, uint16_t b_pin);

		void set_prescaler (uint32_t psc);
		void set_autoreload(uint32_t arl);

		void reset_counter(void);
		uint16_t get_counter(void);
		void delay_us(uint32_t us);
		void delay_ms(uint32_t ms);

		return_t register_event_handler(void(*function_ptr)(tim_event_t event, void *param), void *param = NULL);
		return_t unregister_event_handler(void);

		return_t start(void);
		return_t stop(void);

		return_t start_it(void);
		return_t stop_it(void);

		/* Basic TIMER */
#ifdef ENABLE_DMA
		return_t start_dma(uint32_t *cnt_buffer, uint16_t size);
		return_t stop_dma(void);
#endif
		/* TIMER PWM Mode */
		return_t pwm_set_duty(tim_channel_t channel, uint16_t pwm);
#ifdef ENABLE_DMA
		return_t pwm_start_dma(tim_channel_t channel, uint16_t *pwm, uint16_t size);
		return_t pwm_stop_dma(tim_channel_t channel);
#endif
		/* TIMER Encoder Mode */
		int16_t get_encoder(void);
		void encoder_isr(void);

		void (*handler_callback)(tim_event_t event, void *param) = NULL;
		TIM_TypeDef  *_tim;
		void *parameter = NULL;
		volatile uint32_t counter = 0;

	private:
		tim_config_t *_conf;

		IRQn_Type IRQn;

		void clear_update_isr(void);
};

typedef TIM* tim_t;

void TIM_IRQHandler(tim_t timptr);

#ifdef ENABLE_TIM1
extern tim_t tim1;
#ifndef ENABLE_TIM9
void TIM1_BRK_TIM9_IRQHandler(void);
#endif
#ifndef ENABLE_TIM10
void TIM1_UP_TIM10_IRQHandler(void);
#endif
#ifndef ENABLE_TIM11
void TIM1_TRG_COM_TIM11_IRQHandler(void);
#endif
void TIM1_CC_IRQHandler(void);
#endif

#ifdef ENABLE_TIM2
extern tim_t tim2;
void TIM2_IRQHandler(void);
#endif

#ifdef ENABLE_TIM3
extern tim_t tim3;
void TIM3_IRQHandler(void);
#endif

#ifdef ENABLE_TIM4
extern tim_t tim4;
void TIM4_IRQHandler(void);
#endif

#ifdef ENABLE_TIM5
extern tim_t tim5;
void TIM5_IRQHandler(void);
#endif

#ifdef ENABLE_TIM6
extern tim_t tim6;
void TIM6_DAC_IRQHandler(void);
#endif

#ifdef ENABLE_TIM7
extern tim_t tim7;
void TIM7_IRQHandler(void);
#endif

#ifdef ENABLE_TIM8
extern tim_t tim8;
#ifndef ENABLE_TIM12
void TIM8_BRK_TIM12_IRQHandler(void);
#endif
#ifndef ENABLE_TIM13
void TIM8_UP_TIM13_IRQHandler(void);
#endif
#ifndef ENABLE_TIM14
void TIM8_TRG_COM_TIM14_IRQHandler(void);
#endif
void TIM8_CC_IRQHandler(void);
#endif

#ifdef ENABLE_TIM9
extern tim_t tim9;
void TIM1_BRK_TIM9_IRQHandler(void);
#endif

#ifdef ENABLE_TIM10
extern tim_t tim10;
void TIM1_UP_TIM10_IRQHandler(void);
#endif

#ifdef ENABLE_TIM11
extern tim_t tim11;
void TIM1_TRG_COM_TIM11_IRQHandler(void);
#endif

#ifdef ENABLE_TIM12
extern tim_t tim12;
void TIM8_BRK_TIM12_IRQHandler(void);
#endif

#ifdef ENABLE_TIM13
extern tim_t tim13;
void TIM8_UP_TIM13_IRQHandler(void);
#endif

#ifdef ENABLE_TIM14
extern tim_t tim14;
void TIM8_TRG_COM_TIM14_IRQHandler(void);
#endif

#ifdef __cplusplus
}
#endif

#endif



#endif /* TIM_H_ */
