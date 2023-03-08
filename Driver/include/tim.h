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


// BASE TIMER ENUM - STRUCT.
typedef enum{
	TIM_CHANNEL1 = 0U,
	TIM_CHANNEL2,
	TIM_CHANNEL3,
	TIM_CHANNEL4,
	TIM_NOCHANNEL,
} tim_channel_t;

typedef enum{
	TIM_COUNTER_UP = 0U,
	TIM_COUNTER_DOWN,
} tim_direction_t;

typedef enum{
	TIM_EDGE_ALIGN = 0U,
	TIM_CENTER_MODE1,
	TIM_CENTER_MODE2,
	TIM_CENTER_MODE3,
}tim_align_t;

typedef enum{
	TIM_ARP_DISABLE = 0U,
	TIM_ARP_ENABLE,
} tim_arpe_t;

typedef enum{
	TIM_RISING_EDGE = 0U,
	TIM_FALLING_EDGE = 0x01U,
	TIM_BOTH_EDGE = 0x05U,
}tim_polarity_t;

typedef enum{
	TIM_INTERRUPT_DISABLE = 0U,
	TIM_INTERRUPT_ENABLE = 1U,
} tim_interrupt_t;


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
	tim_align_t     align = TIM_EDGE_ALIGN;
	tim_arpe_t		autoreloadpreload = TIM_ARP_DISABLE;
	tim_interrupt_t interrupt = TIM_INTERRUPT_DISABLE;
	uint32_t        interruptpriority = 0U;
#ifdef ENABLE_DMA
	dma_t 			dma_upd = NULL;
	dma_t           dma_ch1 = NULL;
	dma_t 			dma_ch2 = NULL;
	dma_t           dma_ch3 = NULL;
	dma_t 			dma_ch4 = NULL;
#endif
} tim_config_t;


// TIMER PWM MODE ENUM - STRUCT.
typedef enum{
	TIM_PWM_NOINVERT = 6U,
	TIM_PWM_INVERT = 7U,
} tim_pwm_invert_t;

typedef enum{
	TIM_PRELOAD_DISABLE = 0U,
	TIM_PRELOAD_ENABLE,
} tim_preload_t;

typedef enum{
	TIM_FASTMODE_DISABLE = 0U,
	TIM_FASTMODE_ENABLE,
} tim_fast_mode_t;


typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
	tim_pwm_invert_t invert = TIM_PWM_NOINVERT;
	tim_preload_t preload = TIM_PRELOAD_DISABLE;
	tim_fast_mode_t fastmode = TIM_FASTMODE_DISABLE;
	// Input mode.
	tim_polarity_t polarity = TIM_RISING_EDGE;
	uint8_t ch1_filter = 0U;
	uint8_t ch2_filter = 0U;
} tim_pwm_t;

// TIMER ENCODER MODE ENUM - STRUCT.
typedef enum {
	TIM_ENCODER_MODE1 = 1U,
	TIM_ENCODER_MODE2 = 2U,
	TIM_ENCODER_MODE3 = 3U,
} tim_encoder_mode_t;

typedef struct {
	tim_encoder_mode_t mode = TIM_ENCODER_MODE1;
	GPIO_TypeDef *encA_ch1_port = NULL;
	uint16_t encA_ch1_pin = 0U;
	GPIO_TypeDef *encB_ch2_port = NULL;
	uint16_t encB_ch2_pin = 0U;
	tim_polarity_t encA_ch1_edge = TIM_RISING_EDGE;
	tim_polarity_t encB_ch2_edge = TIM_RISING_EDGE;
	uint8_t encA_ch1_filter = 0U;
	uint8_t encB_ch2_filter = 0U;
	uint8_t encA_ch1_prescaler = 0U;
	uint8_t encB_ch2_prescaler = 0U;
} tim_encoder_t;

// TIMER INPUT CAPTURE MODE ENUM - STRUCT.
typedef struct {
	GPIO_TypeDef *port = NULL;
	uint16_t pin = 0;
	tim_polarity_t polarity = TIM_RISING_EDGE;
	uint8_t prescaler = 0;
	uint8_t filter = 0;
} tim_inputcapture_t;

// TIMER OUTPUT COMPARE MODE ENUM - STRUCT.
typedef enum {
	TIM_FROZEN_MODE = 0x00U,
	TIM_ACTIVE_MODE = 0x01U,
	TIM_INACTIVE_MODE = 0x02U,
	TIM_TOGGLE_MODE = 0x03U,
	TIM_FORCED_ACTIVE_MODE = 0x05U,
	TIM_FORCED_INACTIVE_MODE = 0x04U,
} tim_outputcompare_mode_t;

typedef enum {
	TIM_LEVEL_POLARITY_HIGH,
	TIM_LEVEL_POLARITY_LOW,
} tim_level_polarity_t;

typedef struct {
	tim_outputcompare_mode_t mode = TIM_FROZEN_MODE;
	GPIO_TypeDef *port = NULL;
	uint16_t pin = 0U;
	tim_preload_t preload = TIM_PRELOAD_DISABLE;
	tim_level_polarity_t level_polarity = TIM_LEVEL_POLARITY_HIGH;
} tim_outputcompare_t;


class TIM{
	public:
		TIM(TIM_TypeDef *Timer);

		return_t init(tim_config_t *conf);

		tim_config_t *get_config(void);


		void set_prescaler (uint32_t psc);
		void set_autoreload(uint32_t arl);

		void reset_counter(void);
		uint16_t get_counter(void);
		void delay_us(uint32_t us);
		void delay_ms(uint32_t ms);

		return_t register_event_handler(void(*function_ptr)(tim_channel_t channel, tim_event_t event, void *param), void *param = NULL);
		return_t unregister_event_handler(void);

/* TIMER basic */
		return_t start(void);
		return_t stop(void);

		return_t start_it(void);
		return_t stop_it(void);

#ifdef ENABLE_DMA
		return_t start_dma(uint32_t *cnt_buffer, uint16_t size = 1);
		return_t stop_dma(void);
#endif

/* TIMER PWM output mode */
		return_t set_mode_pwm_output(tim_channel_t channel, tim_pwm_t *conf);

		return_t pwm_output_start(tim_channel_t channel, uint32_t pwm);
		return_t pwm_output_stop(tim_channel_t channel);

		return_t pwm_output_start_it(tim_channel_t channel, uint32_t pwm);
		return_t pwm_output_stop_it(tim_channel_t channel);
#ifdef ENABLE_DMA
		return_t pwm_output_start_dma(tim_channel_t channel, uint32_t *pwm_buffer, uint16_t size = 1);
		return_t pwm_output_stop_dma(tim_channel_t channel);
#endif
		return_t pwm_output_set_duty(tim_channel_t channel, uint32_t pwm);


/* TIMER PWM input mode */
		return_t set_mode_pwm_input(tim_channel_t channel, tim_pwm_t *conf);

		return_t pwm_input_start(tim_channel_t channel);
		return_t pwm_input_stop(tim_channel_t channel);

		return_t pwm_input_start_it(tim_channel_t channel);
		return_t pwm_input_stop_it(tim_channel_t channel);
#ifdef ENABLE_DMA
		return_t pwm_input_start_dma(tim_channel_t channel, uint32_t *pwm_buffer, uint16_t size = 1);
		return_t pwm_input_stop_dma(tim_channel_t channel);
#endif

/* TIMER encoder mode */
		return_t set_mode_encoder(tim_encoder_t *conf);

		return_t encoder_start(void);
		return_t encoder_stop(void);

		return_t encoder_start_it(void);
		return_t encoder_stop_it(void);

#ifdef ENABLE_DMA
		return_t encoder_start_dma(uint32_t *encA_buffer = NULL, uint32_t *encB_buffer = NULL, uint16_t size = 1);
		return_t encoder_stop_dma(void);
#endif
		uint32_t encoder_get_base_counter(void);
		int16_t encoder_get_counter(void);

/* TIMER input capture mode */
		return_t set_mode_inputcapture(tim_channel_t channel, tim_inputcapture_t *conf);

		return_t inputcapture_start(tim_channel_t channel);
		return_t inputcapture_stop(tim_channel_t channel);

		return_t inputcapture_start_it(tim_channel_t channel);
		return_t inputcapture_stop_it(tim_channel_t channel);

#ifdef ENABLE_DMA
		return_t inputcapture_start_dma(tim_channel_t channel, uint32_t *capture_buffer, uint16_t size);
		return_t inputcapture_stop_dma(tim_channel_t channel);
#endif
		uint32_t get_capture_counter(tim_channel_t channel);

/* TIMER output compare mode */
		return_t set_mode_outputcompare(tim_channel_t channel, tim_outputcompare_t *conf);

		return_t outputcompare_start(tim_channel_t channel, uint32_t value);
		return_t outputcompare_stop(tim_channel_t channel);

		return_t outputcompare_start_it(tim_channel_t channel, uint32_t value);
		return_t outputcompare_stop_it(tim_channel_t channel);
#ifdef ENABLE_DMA
		return_t outputcompare_start_dma(tim_channel_t channel, uint32_t *oc_buffer, uint16_t size = 1);
		return_t outputcompare_stop_dma(tim_channel_t channel);
#endif

		return_t set_pulse(tim_channel_t channel, uint32_t pulse);


		void (*handler_callback)(tim_channel_t channel, tim_event_t event, void *param) = NULL;
		TIM_TypeDef  *_tim;
		void *parameter = NULL;
		volatile uint32_t counter = 0;

	private:
		tim_config_t *_conf = NULL;
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
