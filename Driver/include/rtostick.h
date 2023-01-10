/*
 * rtostick.h
 *
 *  Created on: Jan 6, 2023
 *      Author: anh
 */

#ifndef RTOSTICK_H_
#define RTOSTICK_H_

#ifdef __cplusplus
extern "C"{
#endif
#include "sdkconfig.h"

#if (RTOS)


void tim_for_tick_init(uint32_t tick_priority);

void TIM_TICK_IRQ_HANDLER(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* RTOSTICK_H_ */
