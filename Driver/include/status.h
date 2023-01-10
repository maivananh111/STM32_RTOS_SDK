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
} FlagLevel_t;

typedef enum{
	ERR = 0,
	OKE,
	TIMEOUT,
	NOSUPPORT,
	BUSY,
	READY
} Status_t;


typedef struct{
	Status_t Status;
	uint32_t CodeLine;
} Result_t;

enum{
	NO_TIMEOUT = 0,
	DEFAULT_TIMEOUT = 1000U,
};


Status_t check_flag_in_register(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t StatusCheck);

void wait_flag_in_register(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level);
Result_t wait_flag_in_register_timeout(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level, uint16_t TimeOut);
Result_t wait_check_flag_in_register_timeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, FlagLevel_t LevelCheck,
									  __IO uint32_t *RegisterWait, uint32_t FlagWait, FlagLevel_t LevelWait,uint16_t TimeOut);


void set_result(Result_t *res, Status_t Status, uint32_t CodeLine);
void set_result_line(Result_t *res, uint16_t line);


bool result_is_err(Result_t *res);
bool result_is_oke(Result_t *res);
bool result_is_timeout(Result_t *res);
bool result_is_nosupport(Result_t *res);
bool result_is_busy(Result_t *res);
bool result_is_ready(Result_t *res);



#ifdef __cplusplus
}
#endif

#endif /* STATUS_H_ */
