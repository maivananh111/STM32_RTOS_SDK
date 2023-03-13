/**
 * app_main.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "iwdg.h"
#include "stm_log.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "tim.h"
#include "usart.h"

#include "event_groups.h"



static const char *TAG = "MAIN";

QueueHandle_t exti_queue = NULL;
QueueHandle_t usart_queue = NULL;

dma_config_t dma1_stream0_channel6_conf = {
	.stream = DMA1_Stream0,
	.channel = DMA_Channel6,
	.direction = DMA_MEM_TO_PERIPH,
	.mode = DMA_CIRCULAR,
	.datasize = DMA_DATA32BIT,
	.interruptselect = DMA_TRANSFER_COMPLETE_INTERRUPT,
	.interruptpriority = 5,
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

usart_config_t usart1_conf = {
	.baudrate = 115200,
	.control  = USART_INTERRUPT_CONTROl,
	.interruptselect = USART_RECEIVE_INTERRUPT,
	.interruptpriority = 5,
	.txport = GPIOB,
	.txpin = 6,
	.rxport = GPIOB,
	.rxpin = 7,
};

return_t ret;

void task_blink(void *param);
void task_exti(void *param);
void task_pwm(void *param);
void task_usart1(void *param);

void exti_event_handler(uint16_t, void *);
void usart1_event_handler(usart_event_t, void*);

uint32_t tim5_ch3_pwm = 999;

void app_main(void){
	gpio_port_clock_enable(GPIOE);

	exti_init(GPIOE, 3, EXTI_FALLING_EDGE, 4);
	gpio_set_pullup(GPIOE, 3);

	exti_init(GPIOE, 2, EXTI_FALLING_EDGE, 4);
	gpio_set_pullup(GPIOE, 2);

	exti_register_event_handler(exti_event_handler, NULL);

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

	usart1->init(&usart1_conf);
	usart1->register_event_handler(usart1_event_handler, NULL);

	ret = dma1_stream0->init(&dma1_stream0_channel6_conf);
	if(!is_oke(&ret)) STM_LOGE(TAG, "DMA1 Stream0 Channel6 initialize fail.");

	ret = tim5->init(&tim5_conf);
	if(!is_oke(&ret)) STM_LOGE(TAG, "TIM5 initialize fail.");
	tim5->set_mode_pwm_output(TIM_CHANNEL3, &tim5_ch3_conf);
	tim5->pwm_output_start_dma(TIM_CHANNEL3, &tim5_ch3_pwm, 1);

	exti_queue = xQueueCreate(30, sizeof(char*));
	usart_queue = xQueueCreate(5, sizeof(char*));


	xTaskCreate(task_blink, "task_blink", byte_to_word(1024), NULL, 2, NULL);
	xTaskCreate(task_exti, "task_exti", byte_to_word(1024), NULL, 4, NULL);
	xTaskCreate(task_pwm, "task_pwm", byte_to_word(1024), NULL, 2, NULL);
	xTaskCreate(task_usart1, "task_usart", byte_to_word(2048), NULL, 10, NULL);


	while(1){
		STM_LOGW(TAG, "app_main running.");
		vTaskDelay(500);
	}
}

void task_blink(void *param){

	while(1){
		gpio_set(GPIOC, 13);
		vTaskDelay(10);
		STM_LOGM(TAG, "heap: %lu", stm_get_free_heap_size());
		gpio_reset(GPIOC, 13);
		vTaskDelay(10000);
	}
}

void task_exti(void *param){
	uint16_t i = 0;;

	while(1){
		if(xQueueReceive(exti_queue, &i, 10) == pdPASS){
			STM_LOGI(TAG, "External interrupt line %d", i);
		}
		else{
			STM_LOGE(TAG, "External interrupt Queue Empty.");
		}
		vTaskDelay(500);
	}
}

void task_pwm(void *param){

	while(1){
		tim5_ch3_pwm--;
		if(tim5_ch3_pwm == 0) tim5_ch3_pwm = 999;
		vTaskDelay(2);
	}
}

void task_usart1(void *param){
	uint8_t *usart1_data;
	usart1->receive_to_idle_start_it(30);

	while(1){
		if(xQueueReceiveFromISR(usart_queue, &usart1_data, NULL) == pdPASS){
			STM_LOGI(TAG, "USART RX: %s", (char *)usart1_data);
			free(usart1_data);
		}
		vTaskDelay(10);
	}
}

void exti_event_handler(uint16_t pin, void *param){
	uint16_t i = pin;
	BaseType_t xTaskWokenByReceive;

	STM_LOGR(TAG, "Line %d interrupt.", i);
	if(exti_queue != NULL){
		if(xQueueSendFromISR(exti_queue, &i, &xTaskWokenByReceive) == pdPASS) {
			portEND_SWITCHING_ISR(xTaskWokenByReceive);
		}
		else {
			STM_LOGW(TAG, "Queue send failed.");
		}
	}
}

void usart1_event_handler(usart_event_t event, void *param){

	if(event == USART_EVENT_IDLE_STATE || event == USART_EVENT_BUFFER_OVERFLOW){
		uint8_t *tmp;
		usart1->get_buffer(&tmp);

 		if(usart_queue != NULL){
			if(xQueueSendFromISR(usart_queue, &tmp, NULL) == pdPASS) {
				portEND_SWITCHING_ISR(NULL);
			}
			else {
				STM_LOGW(TAG, "Queue send failed.");
			}
		}
	}
}












