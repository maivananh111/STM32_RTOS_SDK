/**
 * app_main.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "main.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "exti.h"
#include "stdlib.h"


static const char *TAG = "MAIN";
char *aloc_buf = NULL;

QueueHandle_t exti_queue = NULL;
volatile uint32_t cnt = 0;

void task_blink(void *param);
void task_logmem(void *param);
void task_exti(void *param);

void exti_eventhandler(uint16_t pin, void *param);

int app_main(void){
	gpio_port_clock_enable(GPIOE);

	exti_init(GPIOE, 3, EXTI_FALLING_EDGE, 4);
	gpio_set_pullup(GPIOE, 3);

	exti_init(GPIOE, 2, EXTI_FALLING_EDGE, 4);
	gpio_set_pullup(GPIOE, 2);

	exti_register_event_handler(exti_eventhandler, NULL);


	gpio_port_clock_enable(GPIOC);
	GPIO_Config_t pc13 = {
		.port = GPIOC,
		.pin = 13,
		.direction = GPIO_Output,
		.outputtype = GPIO_PushPull,
		.outputspeed = GPIO_Speed_VeryHigh,
		.pullresister = GPIO_NoPull,
	};
	gpio_init(&pc13);

	exti_queue = xQueueCreate(30, sizeof(char*));
	if(exti_queue == 0) STM_LOGE(TAG, "Queue create failed.");
	else STM_LOGI(TAG, "Queue create oke.");



	xTaskCreate(task_logmem, "task_logmem", 1024, NULL, 3, NULL);
	xTaskCreate(task_blink, "task_blink", 1024, NULL, 2, NULL);
	xTaskCreate(task_exti, "task_exti", 1024, NULL, 4, NULL);

	vTaskStartScheduler();

	return 0;
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
		aloc_buf = (char *)malloc(10);
		STM_LOGM(TAG, "Free heap size: %lu", get_free_heap_size());
		vTaskDelay(1000);
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

















