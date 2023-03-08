/**
 * app_main.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "main.h"
#include "stdlib.h"

#include "gpio.h"
#include "exti.h"
#include "tim.h"


static const char *TAG = "MAIN";

QueueHandle_t exti_queue = NULL;


dma_config_t dma1_stream0_channel6_conf = {
	.stream = DMA1_Stream0,
	.channel = DMA_Channel6,
	.direction = DMA_MEM_TO_PERIPH,
	.mode = DMA_CIRCULAR,
	.datasize = DMA_DATA32BIT,
	.interruptselect = DMA_TRANSFER_COMPLETE_INTERRUPT,
	.interruptpriority = 5,
};

tim_config_t tim3_conf = {
	.prescaler = 42000,
	.reload = 60000,
	.interrupt = TIM_INTERRUPT_ENABLE,
	.interruptpriority = 6,
};

tim_config_t tim5_conf = {
	.prescaler = 84,
	.reload = 1000,
	.dma_ch3 = dma1_stream0,
};
tim_pwm_t tim5_ch3_conf = {
	.port = GPIOA,
	.pin = 2,
	.invert = TIM_PWM_NOINVERT,
	.preload = TIM_PRELOAD_ENABLE,
	.fastmode = TIM_FASTMODE_ENABLE,
};
uint32_t *aloc_buf;

return_t ret;

void task_blink(void *param);
void task_logmem(void *param);
void task_exti(void *param);
void task_pwm(void *param);

void exti_eventhandler(uint16_t pin, void *param);
void tim3_eventhandler(tim_channel_t channel, tim_event_t event, void *param);
void Dma1_Stream0_eventhandler(void *Parameter, dma_event_t event);

uint32_t tim5_ch3_pwm = 1;

void app_main(void){
	gpio_port_clock_enable(GPIOE);

	exti_init(GPIOE, 3, EXTI_FALLING_EDGE, 4);
	gpio_set_pullup(GPIOE, 3);

	exti_init(GPIOE, 2, EXTI_FALLING_EDGE, 4);
	gpio_set_pullup(GPIOE, 2);

	exti_register_event_handler(exti_eventhandler, NULL);

	gpio_port_clock_enable(GPIOC);
	gpio_config_t pc13 = {
		.port = GPIOC,
		.pin = 13,
		.direction = GPIO_Output,
		.outputtype = GPIO_PushPull,
		.outputspeed = GPIO_Speed_VeryHigh,
		.pullresister = GPIO_NoPull,
	};
	gpio_init(&pc13);

	ret = tim3->init(&tim3_conf);
	if(!is_oke(&ret)) STM_LOGE(TAG, "TIM3 initialize fail.");
	tim3->register_event_handler(tim3_eventhandler, NULL);
	ret = tim3->start_it();
	if(!is_oke(&ret)) STM_LOGE(TAG, "TIM3 start fail.");


	ret = dma1_stream0->init(&dma1_stream0_channel6_conf);
	if(!is_oke(&ret)) STM_LOGE(TAG, "DMA1 Stream0 Channel6 initialize fail.");
	dma1_stream0->register_event_handler(Dma1_Stream0_eventhandler, NULL);

	ret = tim5->init(&tim5_conf);
	if(!is_oke(&ret)) STM_LOGE(TAG, "TIM5 initialize fail.");
	tim5->set_mode_pwm_output(TIM_CHANNEL3, &tim5_ch3_conf);
	tim5->pwm_output_start_dma(TIM_CHANNEL3, &tim5_ch3_pwm, 1);

	exti_queue = xQueueCreate(30, sizeof(char*));
	if(exti_queue == NULL) STM_LOGE(TAG, "Queue create failed.");
	else STM_LOGI(TAG, "Queue create oke.");


	xTaskCreate(task_logmem, "task_logmem", byte_to_word(1024), NULL, 3, NULL);
	xTaskCreate(task_blink, "task_blink", byte_to_word(1024), NULL, 2, NULL);
	xTaskCreate(task_exti, "task_exti", byte_to_word(1024), NULL, 4, NULL);
	xTaskCreate(task_pwm, "task_pwm", byte_to_word(1024), NULL, 2, NULL);

}

void task_blink(void *param){

	while(1){
		gpio_set(GPIOC, 13);
		vTaskDelay(10);
		STM_LOGD(TAG, "Task blink led.");
		gpio_reset(GPIOC, 13);
		vTaskDelay(5000);

	}
}

void task_logmem(void *param){

	while(1){
		aloc_buf = (uint32_t *)malloc(1000*sizeof(uint32_t));
		if(aloc_buf)
		STM_LOGM(TAG, "Free heap size: %lu", get_free_heap_size());
		vTaskDelay(1000);
		free(aloc_buf);
	}
}

void task_exti(void *param){
	uint16_t i = 0;;

	while(1){
		if(xQueueReceive(exti_queue, &i, portMAX_DELAY) == pdPASS){
			STM_LOGI(TAG, "External interrupt line %d", i);
		}
		else{
			STM_LOGE(TAG, "Queue empty.");
		}
		vTaskDelay(50);
	}
}

void task_pwm(void *param){

	while(1){
		tim5_ch3_pwm++;
		if(tim5_ch3_pwm == 999) tim5_ch3_pwm = 0;
//		STM_LOGI(TAG, "PWM Value = %d", tim2_ch3_pwm);
//		tim2->pwm_set_duty(TIM_CHANNEL3, tim5_ch3_pwm);
		vTaskDelay(2);
	}

}


void exti_eventhandler(uint16_t pin, void *param){
	uint16_t i = pin;
	BaseType_t xTaskWokenByReceive;

	STM_LOGR(TAG, "Line %d interrupt.", i);
	if(exti_queue != NULL){
		if(xQueueSendFromISR(exti_queue, &i, &xTaskWokenByReceive) == pdPASS) {
			STM_LOGI(TAG, "Queue sended oke.");
			portEND_SWITCHING_ISR(xTaskWokenByReceive);
		}
		else {
			STM_LOGW(TAG, "Queue send failed.");
		}
	}
}

void tim3_eventhandler(tim_channel_t channel, tim_event_t event, void *param){
	if(event == TIM_UPDATE_EVENT){
		STM_LOGI(TAG, "Timer 3 update event.");
	}
}

void Dma1_Stream0_eventhandler(void *Parameter, dma_event_t event){

}













