/*
 * status.h
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */

#ifndef STATUS_H_
#define STATUS_H_


#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"
#include "stm32f4xx.h"

#include "sdkconfig.h"


typedef enum{
	FLAG_RESET = 0,
	FLAG_SET,
} flaglevel_t;

typedef enum{
	ERR = 0,
	OKE,
	TIMEOUT,
	UNSUPPORTED,
	BUSY,
	READY,
	NOTAVAILABLE,
} status_t;


typedef struct{
	status_t Status = OKE;
	uint32_t Line = 0U;
} return_t;

enum{
	NO_TIMEOUT = 0,
	DEFAULT_TIMEOUT = 1000U,
};


status_t check_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t StatusCheck);

void wait_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level);
return_t wait_flag_in_register_timeout(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level, uint16_t TimeOut);
return_t wait_check_flag_in_register_timeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, flaglevel_t LevelCheck,
									  __IO uint32_t *RegisterWait, uint32_t FlagWait, flaglevel_t LevelWait,uint16_t TimeOut);


void set_return(return_t *res, status_t Status, uint32_t CodeLine);
void set_return_line(return_t *res, uint16_t line);


bool is_err(return_t *res);
bool is_oke(return_t *res);
bool is_timeout(return_t *res);
bool is_unsupported(return_t *res);
bool is_busy(return_t *res);
bool is_ready(return_t *res);
bool is_unavailable(return_t *res);


#ifdef __cplusplus
}
#endif

#endif /* STATUS_H_ */
