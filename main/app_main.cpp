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


static const char *TAG = "MAIN";


void task_blink(void *param);
void task_logmem(void *param);


extern "C"{
	int app_main(void){

		xTaskCreate(task_blink, "task_blink", 1024, NULL, 2, NULL);
		xTaskCreate(task_logmem, "task_logmem", 2048, NULL, 3, NULL);

		vTaskStartScheduler();
		while(1){

			STM_LOGD(TAG, "Task in while(1).");
			vTaskDelay(1000);
		}
	}
}



void task_blink(void *param){
	gpio_port_clock_enable(GPIOC);
	gpio_set_mode(GPIOC, 13, GPIO_OUTPUT_PUSHPULL);

	while(1){
		gpio_set(GPIOC, 13);
		vTaskDelay(100);
		STM_LOGD(TAG, "Task blink led.");
		gpio_reset(GPIOC, 13);
		vTaskDelay(100);
	}
}

void task_logmem(void *param){

	while(1){
		STM_LOGI(TAG, "Free heap size: %lu", get_free_heap_size());
		vTaskDelay(1000);
	}
}
