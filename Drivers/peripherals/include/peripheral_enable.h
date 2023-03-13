/*
 * periph_en.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPH_EN_H_
#define PERIPH_EN_H_



#define ENABLE_EXTI         1
#define ENABLE_TIM			1
#define ENABLE_DMA			1
#define ENABLE_USART		1
#define ENABLE_SPI			1
#define ENABLE_I2C			1
#define ENABLE_IWDG			1

/**
 *  EXTI ENABLE
 */
#if ENABLE_EXTI
//#define ENABLE_EXTI0
//#define ENABLE_EXTI1
#define ENABLE_EXTI2
#define ENABLE_EXTI3
//#define ENABLE_EXTI4
//#define ENABLE_EXTI9_5
//#define ENABLE_EXTI15_10
#endif /* ENABLE_EXTI */

/**
 * DMA STREAM ENABLE.
 */
#if ENABLE_DMA
#define ENABLE_DMA1_STREAM0
//#define ENABLE_DMA1_STREAM1
//#define ENABLE_DMA1_STREAM2
//#define ENABLE_DMA1_STREAM3
//#define ENABLE_DMA1_STREAM4
//#define ENABLE_DMA1_STREAM5
//#define ENABLE_DMA1_STREAM6
//#define ENABLE_DMA1_STREAM7
//
//#define ENABLE_DMA2_STREAM0
//#define ENABLE_DMA2_STREAM1
//#define ENABLE_DMA2_STREAM2
//#define ENABLE_DMA2_STREAM3
//#define ENABLE_DMA2_STREAM4
//#define ENABLE_DMA2_STREAM5
//#define ENABLE_DMA2_STREAM6
//#define ENABLE_DMA2_STREAM7
#endif /* ENABLE_DMA */

/**
 * TIM ENABLE
 */
#if ENABLE_TIM
//#define ENABLE_TIM1
//#define ENABLE_TIM2
//#define ENABLE_TIM3
//#define ENABLE_TIM4
#define ENABLE_TIM5
//#define ENABLE_TIM6
//#define ENABLE_TIM7
//#define ENABLE_TIM8
//#define ENABLE_TIM9
//#define ENABLE_TIM10
//#define ENABLE_TIM11
//#define ENABLE_TIM12
//#define ENABLE_TIM13
//#define ENABLE_TIM14
#endif /* ENABLE_TIM */

/**
 * USART ENABLE
 */
#if ENABLE_USART
#define ENABLE_USART1
#define ENABLE_USART2
#define ENABLE_USART3
#define ENABLE_UART4
#define ENABLE_UART5
#define ENABLE_USART6
#endif /* ENABLE_USART */

#endif /* PERIPH_EN_H_ */
