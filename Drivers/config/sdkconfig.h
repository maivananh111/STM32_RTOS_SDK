/*
 * sdkconfig.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef SDKCONFIG_H_
#define SDKCONFIG_H_


#define SDK_VERSION                     "v1.0.1"

/**
 * Device and operating system set.
 */
/* Core */
//#define Cortex_M0
//#define Cortex_M3
#define Cortex_M4
/* Device */
#define STM32F407xx            			// Define STM32 device.

/**
 * RTOS Configuration.
 */
#define TOTAL_HEAP_SIZE                 	 (128U * 1024U)
#define RTOS_HEAP_SIZE                  	 (20 * 1024U)
#define APP_MAIN_TASK_SIZE              	 byte_to_word(4096)
#define APP_MAIN_TASK_PRIO              	 1


#define RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY   4 // Your system interrupt priority must be greater than this number.
#define RTOS_TASK_MAX_PRIORITY               32
#define RTOS_TICK_RATE                       SYSTICK_RATE
#define byte_to_word(x)                      (x/4)
#define RTOS_IWDG                            1
/**
 * Embedded flash memory configuration.
 */
#define FLASH_DATA_CACHE					 1
#define FLASH_INSTRUCTION_CACHE         	 1
#define FLASH_PREFETCH_MODE             	 1

/**
 * System tick configuration.
 */
#define SYSTICK_RATE                    1000U // 1000 for 1ms.
#define SYSTICK_PRIORITY                15U

/**
 * Clock configuration.
 */
#define HSE_VALUE 						25000000U // Hz
#define HSI_VALUE 						16000000U // Hz
#define HSI_TRIM_VALUE                  16U
#define OVER_CLOCK                      1 // Note: if over clock, your device maybe not work correctly

#define LSI_VALUE                       32000U
#define LSE_VALUE                       32768U
// Constant value. Don't change this.
#define SYSTEM_CLOCK_FREQUENCY_MAX      168000000U // Hz
#define AHB_CLOCK_FREQUENCY_MAX         168000000U // Hz (AHB is HCLK)
#define APB1_CLOCK_FREQUENCY_MAX        42000000U  // Hz (APB1 is PCLK1)
#define APB2_CLOCK_FREQUENCY_MAX        84000000U  // Hz (APB1 is PCLK2)

// Configuration value, you need to set up them.
#define OSC_CLOCK_SOURCE                HSE_CRYSTAL
#define SYSTEM_CLOCK_MUX                PLLCLK
#define PLL_SOURCE_MUX                  PLL_SOURCE_HSE // if SYSTEM_CLOCK_MUX is not PLLCLK, ignore this value.

#define PLLM                            25U
#define PLLN                            336U
#define PLLP                            2U
#define PLLQ                            4U

#define SYSTEM_CLOCK_FREQUENCY          168000000U  // Hz
#define AHB_CLOCK_FREQUENCY          	168000000U  // Hz
#define AHB_PRESCALER          			Clock_Div_1 // Hz (AHB is HCLK)
#define APB1_PRESCALER          		Clock_Div_4 // Hz (APB1 is PCLK1)
#define APB2_PRESCALER          		Clock_Div_2 // Hz (APB1 is PCLK2)


/**
 * Log monitor.
 */
#define LOG_MONITOR						1
#if LOG_MONITOR
#define LOG_UART_BAUDRATE               115200U
#define LOG_UART_NUM 					USART1

#define LOG_UART_TX 					6
#define LOG_UART_TXP 					GPIOB
#define LOG_UART_RX 					7
#define LOG_UART_RXP 					GPIOB

#define LOG_TIME 						1
#endif



/**
 * PERIPHERAL CONFIG.
 */
#define CHIP_RESET_IF_CONFIG_FAIL 1
#if CHIP_RESET_IF_CONFIG_FAIL
#define WAIT_FOR_RESET_TIME 5
#endif /* CHIP_RESET_IF_CONFIG_FAIL */





















#endif /* SDKCONFIG_H_ */
