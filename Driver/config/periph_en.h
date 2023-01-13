/*
 * periph_en.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPH_EN_H_
#define PERIPH_EN_H_



#define ENABLE_EXTI
#define ENABLE_GPIO
#define ENABLE_TIM
#define ENABLE_DMA
#define ENABLE_USART
#define ENABLE_I2C

/* EXTI ENABLE */
#ifdef ENABLE_EXTI
//#define ENABLE_EXTI0
//#define ENABLE_EXTI1
#define ENABLE_EXTI2
#define ENABLE_EXTI3
//#define ENABLE_EXTI4
//#define ENABLE_EXTI9_5
//#define ENABLE_EXTI15_10

#endif

#endif /* PERIPH_EN_H_ */
