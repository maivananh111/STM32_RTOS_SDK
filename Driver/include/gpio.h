/*
 * gpio.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "sdkconfig.h"

#include "stdio.h"
#include "stm32f4xx.h"
#include "periph_en.h"

#ifdef ENABLE_GPIO

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_OUTPUTSPEED_DEFAULT GPIO_Speed_VeryHigh

typedef enum{
	GPIO_Input,
	GPIO_Output,
	GPIO_AlternateFunction,
} GPIO_Direction_t;

typedef enum{
	GPIO_PushPull,
	GPIO_OpenDrain,
} GPIO_type_t;

typedef enum{
	GPIO_Speed_Low,
	GPIO_Speed_Medium,
	GPIO_Speed_High,
	GPIO_Speed_VeryHigh,
} GPIO_OutputSpeed_t;

typedef enum{
	GPIO_NoPull,
	GPIO_PullUp,
	GPIO_PullDown,
} GPIO_Pull_t;

typedef enum{
	GPIO_INPUT,
	GPIO_INPUT_PULLUP,
	GPIO_INPUT_PULLDOWN,

	GPIO_OUTPUT_OPENDRAIN,
	GPIO_OUTPUT_OPENDRAIN_PULLUP,
	GPIO_OUTPUT_OPENDRAIN_PULLDOWN,
	GPIO_OUTPUT_PUSHPULL,
	GPIO_OUTPUT_PUSHPULL_PULLUP,
	GPIO_OUTPUT_PUSHPULL_PULLDOWN,

	GPIO_ANALOG,
} GPIO_Mode_t;

typedef enum{
	AF0_SYSTEM,
	AF1_TIM1_2,
	AF2_TIM3_5,
	AF3_TIM8_11,
	AF4_I2C1_3,
	AF5_SPI1_2,
	AF6_SPI3,
	AF7_USART1_3,
	AF8_USART4_6,
	AF9_CAN1_2_TIM12_14,
	AF10_USB,
	AF11_ETH,
	AF12_FSMC_SDIO_USB,
	AF13_DCMI,
	AF14,
	AF15_EVENTOUT,
} GPIO_AlternateFunction_t;

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
	GPIO_Direction_t direction;
	GPIO_type_t outputtype;
	GPIO_OutputSpeed_t outputspeed;
	GPIO_Pull_t pullresister;
	GPIO_AlternateFunction_t function;
} GPIO_Config_t;


void gpio_allport_clock_enable(void);
void gpio_port_clock_enable(GPIO_TypeDef *port);

void gpio_init(GPIO_Config_t *conf);
void gpio_deinit(GPIO_TypeDef *port, uint16_t pin);

void gpio_set_mode(GPIO_TypeDef *port, uint16_t pin, GPIO_Mode_t mode);
void gpio_set_alternatefunction(GPIO_TypeDef *port, uint16_t pin, GPIO_AlternateFunction_t function);
void gpio_set_alternatefunction_type(GPIO_TypeDef *port, uint16_t pin, GPIO_Mode_t mode);

void gpio_set_pullup(GPIO_TypeDef *port, uint16_t pin);
void gpio_set_pulldown(GPIO_TypeDef *port, uint16_t pin);

void gpio_set(GPIO_TypeDef *port, uint16_t pin);
void gpio_reset(GPIO_TypeDef *port, uint16_t pin);
void gpio_toggle(GPIO_TypeDef *port, uint16_t pin);

void gpio_set_level(GPIO_TypeDef *port, uint16_t pin, int level);
int gpio_get_level(GPIO_TypeDef *port, uint16_t pin);

#define GPIOA_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN
#define GPIOB_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN
#define GPIOC_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOCEN
#define GPIOD_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN
#define GPIOE_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOEEN
#define GPIOF_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOFEN
#define GPIOG_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOGEN
#define GPIOH_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOHEN
#define GPIOI_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOIEN

#ifdef __cplusplus
}
#endif

#endif

#endif /* GPIO_H_ */
