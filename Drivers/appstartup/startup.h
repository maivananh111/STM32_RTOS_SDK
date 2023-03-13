/*
 * main.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef STARTUP_H_
#define STARTUP_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include "stm32f4xx.h"

#include "task.h"

#include "system.h"
#include "systick.h"
#include "rcc.h"
#include "gpio.h"
#include "iwdg.h"

#include "stm_log.h"

#ifdef LOG_MONITOR
static void uart_log_init(void);
static void uart_log(char *log);
#endif

void app_main_task(void *);

#ifdef __cplusplus
}
#endif

#endif /* STARTUP_H_ */
