/*
 * spi.h
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */

#ifndef SPI_H_
#define SPI_H_

#pragma once
#include "peripheral_enable.h"
#if ENABLE_SPI

#include "stm32f4xx.h"

#include "stdint.h"

#include "status.h"
#include "gpio.h"
#if ENABLE_DMA
#include "dma.h"
#endif /* ENABLE_DMA */

#ifdef __cplusplus
extern "C"{
#endif /* __cplusplus */

#define SPI_LOG_DEBUG_ENABLE
#define SPI_DEFAULT_TIMEOUT 100U

typedef enum{
	SPI_FULLDUPLEX_MASTER = 0U,
	SPI_HALFDUPLEX_MASTER,
	SPI_FULLDUPLEX_SLAVE,
	SPI_HALFDUPLEX_SLAVE,
} spi_mode_t;

typedef enum{
	SPI_NORMAL_CONTROL = 0U,
	SPI_INTERRUPT_CONTROL,
	SPI_DMA_CONTROL,
	SPI_INTERRUPT_DMA_CONTROL
} spi_periph_control_t;

typedef enum{
	SPI_RECEIVE_INTERRUPT,
	SPI_TRANSMIT_INTERRUPT,
	SPI_TRANSMIT_RECEIVE_INTERRUPT,
} spi_interruptselect_t;

typedef enum{
	SPI_DATASIZE_8BIT = 0U,
	SPI_DATASIZE_16BIT,
} spi_datasize_t;

typedef enum{
	SPI_BITORDERING_MSB = 0U,
	SPI_BITORDERING_LSB,
} spi_bitodering_t;

typedef enum{
	SPI_CLOCKDIVISION_2 = 0UL,
	SPI_CLOCKDIVISION_4,
	SPI_CLOCKDIVISION_8,
	SPI_CLOCKDIVISION_16,
	SPI_CLOCKDIVISION_32,
	SPI_CLOCKDIVISION_64,
	SPI_CLOCKDIVISION_128,
	SPI_CLOCKDIVISION_256,
} spi_clockdivision_t;

typedef enum{
	SPI_CLOCK_CPOL0_CPHA0 = 0U,
	SPI_CLOCK_CPOL0_CPHA1,
	SPI_CLOCK_CPOL1_CPHA0,
	SPI_CLOCK_CPOL1_CPHA1,
} spi_clocksample_t;

typedef enum{
	SPI_EVENT_TRANSMIT_COMPLETE,
	SPI_EVENT_RECEIVE_COMPLETE,
	SPI_EVENT_ERROR,
} spi_event_t;

typedef struct {
	spi_mode_t 			  mode            = SPI_FULLDUPLEX_MASTER;
	spi_periph_control_t  control         = SPI_NORMAL_CONTROL;
	spi_interruptselect_t interruptselect = SPI_RECEIVE_INTERRUPT;
	spi_datasize_t 		  datasize        = SPI_DATASIZE_8BIT;
	spi_bitodering_t 	  bitordering  	  = SPI_BITORDERING_MSB;
	spi_clockdivision_t   clockdivision   = SPI_CLOCKDIVISION_4;
	spi_clocksample_t 	  clocksample 	  = SPI_CLOCK_CPOL0_CPHA0;
	uint32_t 			  interruptpriority = 0;
	GPIO_TypeDef 		  *clkport;
	uint16_t 			  clkpin;
	GPIO_TypeDef 		  *misoport;
	uint16_t 			  misopin;
	GPIO_TypeDef 		  *mosiport;
	uint16_t 			  mosipin;
	GPIO_TypeDef 		  *csport;
	uint16_t 			  cspin;
#if ENABLE_DMA
	dma_t 				  txdma = NULL;
	dma_t 				  rxdma = NULL;
#endif /* ENABLE_DMA */
} spi_config_t;

class SPI {
	public:
		SPI(SPI_TypeDef *spi);
		void init(spi_config_t *conf);

		return_t register_event_handler(void (*function_ptr)(spi_event_t event, void *param), void *param = NULL);

		return_t transmit(uint32_t data, uint32_t size);
		return_t receive(uint32_t data, uint32_t size);
		return_t transmit_receive(uint32_t txdata, uint32_t rxdata, uint32_t size);


		return_t transmit_start_it(uint32_t data, uint32_t size);
		return_t receive_start_it(uint32_t data, uint32_t size);
		return_t transmit_receive_start_it(uint32_t txdata, uint32_t rxdata, uint32_t size);

		return_t transmit_stop_it(void);
		return_t receive_stop_it(void);
		return_t transmit_receive_stop_it(void);

#if ENABLE_DMA
		return_t transmit_start_dma(uint32_t data, uint32_t size);
		return_t receive_start_dma(uint32_t data, uint32_t size);
		return_t transmit_receive_start_dma(uint32_t txdata, uint32_t rxdata, uint32_t size);

		return_t transmit_stop_dma(void);
		return_t receive_stop_dma(void);
		return_t transmit_receive_stop_dma(void);

		dma_t _txdma = NULL, _rxdma = NULL;
#endif /* ENABLE_DMA */
		SPI_TypeDef *_spi;
		void *parameter = NULL;
		void (*handler_callback)(spi_event_t event, void *param) = NULL;

	private:
		spi_config_t *_conf = NULL;
		IRQn_Type IRQn;

};

typedef SPI spi_t;



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* ENABLE_SPI */

#endif /* SPI_H_ */
