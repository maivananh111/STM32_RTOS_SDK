
#include "../appstartup/startup.h"

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "math.h"



RCC_Config_t rcc = {
	.hse_frequency = HSE_VALUE,
	.hsi_frequency = HSI_VALUE,
	.hsi_trim = HSI_TRIM_VALUE,
	.osc_source = OSC_CLOCK_SOURCE,
	.sysclock_source = SYSTEM_CLOCK_MUX,
	.pll_source = PLL_SOURCE_MUX,
	.sysclock_frequency = SYSTEM_CLOCK_FREQUENCY,
	.ahb_prescaler = AHB_PRESCALER,
	.apb1_prescaler = APB1_PRESCALER,
	.apb2_prescaler = APB2_PRESCALER,
	rcc.pll.pllm = PLLM,
	rcc.pll.plln = PLLN,
	rcc.pll.pllp = PLLP,
	rcc.pll.pllq = PLLQ,
};

#ifdef LOG_MONITOR
static const char* TAG = (const char *)"STARTUP";
static USART_TypeDef *log_uart = (USART_TypeDef *)LOG_UART_NUM;
#endif


int main(void){
	system_init();
	rcc_init(&rcc);

#if RTOS_IWDG
	iwdg_init(IWDG_PRESCALER_64, 2500);
#endif

	gpio_port_clock_enable(GPIOC);
	gpio_set_mode(GPIOC, 6, GPIO_OUTPUT_PUSHPULL);
	gpio_set(GPIOC, 6);

#ifdef LOG_MONITOR
	uart_log_init();
	stm_log_init(uart_log);

	STM_LOGI(TAG, "SDK version   : %s",     SDK_VERSION);
	STM_LOGW(TAG, "Revision ID   : 0x%04x", get_revid());
	STM_LOGE(TAG, "Device ID     : 0x%04x", get_devid());
	STM_LOGD(TAG, "Core frequency: %luHz",  rcc_get_bus_frequency(SYSCLK));
	STM_LOGM(TAG, "AHB frequency : %luHz",  rcc_get_bus_frequency(AHB));
	STM_LOGP(TAG, "APB1 frequency: %luHz",  rcc_get_bus_frequency(APB1));
	STM_LOGR(TAG, "APB2 frequency: %luHz",  rcc_get_bus_frequency(APB2));

#endif
	xTaskCreate(app_main_task, "app_main_task", APP_MAIN_TASK_SIZE, NULL, APP_MAIN_TASK_PRIO, NULL);
	STM_LOGI(TAG, "Starting scheduler on CPU.");
	vTaskStartScheduler();
}


void app_main_task(void *param){
	STM_LOGI(TAG, "Calling app_main().");
	extern void app_main(void);
	app_main();
	STM_LOGI(TAG, "Returned from app_main().");
	vTaskDelete(NULL);
}
extern"C"{
	void vApplicationIdleHook(void){
		/* Do something while cpu idle. */

#ifdef RTOS_IWDG
		/* Reset Independent Watchdog timer */
		iwdg_refresh();
#endif /* RTOS_IWDG */
	}
}

#ifdef LOG_MONITOR
static void uart_log_init(void){
	__IO uint32_t USART_BusFreq = 0UL;

	gpio_port_clock_enable(LOG_UART_TXP);
	gpio_port_clock_enable(LOG_UART_RXP);
	if(log_uart == USART1 || log_uart == USART2 || log_uart == USART3){
		gpio_set_alternatefunction(LOG_UART_TXP, LOG_UART_TX, AF7_USART1_3);
		gpio_set_alternatefunction(LOG_UART_RXP, LOG_UART_RX, AF7_USART1_3);
	}
	else{
		gpio_set_alternatefunction(LOG_UART_TXP, LOG_UART_TX, AF8_USART4_6);
		gpio_set_alternatefunction(LOG_UART_RXP, LOG_UART_RX, AF8_USART4_6);
	}
	if(log_uart == USART1 || log_uart == USART6) {
		if(log_uart == USART1) 		RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
		else if(log_uart == USART6) RCC -> APB2ENR |= RCC_APB2ENR_USART6EN;
		USART_BusFreq = rcc_get_bus_frequency(APB2);
	}
	else {
		if(log_uart == USART2) 		RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
		else if(log_uart == USART3) RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;
		else if(log_uart == UART4)  RCC -> APB1ENR |= RCC_APB1ENR_UART4EN;
		else if(log_uart == UART5)  RCC -> APB1ENR |= RCC_APB1ENR_UART5EN;
		USART_BusFreq = rcc_get_bus_frequency(APB1);
	}

	log_uart -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	float USARTDIV = (float)(USART_BusFreq/(LOG_UART_BAUDRATE * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	DIV_Fraction = ceil(Fraction);
	log_uart -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	uart_log((char *)"\r\n\r\n");
	uart_log((char *)"************-----------------------************\r\n");
}

static void uart_log(char *log){
	while(*log) {
		log_uart -> DR = *log++;
		while(!(log_uart -> SR & USART_SR_TC));
	}
}
#endif












