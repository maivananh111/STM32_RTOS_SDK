/*
 * main.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef MAIN_H_
#define MAIN_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include "stm32f4xx.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "system.h"
#include "systick.h"
#include "rcc.h"
#include "gpio.h"

#include "stm_log.h"


void app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
