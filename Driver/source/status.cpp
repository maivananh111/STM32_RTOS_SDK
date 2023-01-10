/*
 * status.cpp
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */


#include "status.h"
#include "systick.h"
#include "stdlib.h"
#include "string.h"



uint32_t (*GetCounterFunction)(void) = get_tick;


/**
 * @fn Status_t check_flag_in_register(volatile uint32_t*, uint32_t, FlagLevel_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param StatusCheck
 * @return
 */
Status_t check_flag_in_register(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t StatusCheck){
	if((StatusCheck == FLAG_SET)? ((*Register & Flag) != 0U) : ((*Register & Flag) == 0U)) return OKE;
	return ERR;
}

/**
 * @fn void wait_flag_in_register(volatile uint32_t*, uint32_t, FlagLevel_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param Level
 */
void wait_flag_in_register(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level){
	while((Level == FLAG_RESET)?((*Register & Flag)) : (!(*Register & Flag)));
}

/**
 * @fn Result_t wait_flag_in_register_timeout(volatile uint32_t*, uint32_t, FlagLevel_t, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param Level
 * @param TimeOut
 * @return
 */
Result_t wait_flag_in_register_timeout(__IO uint32_t *Register, uint32_t Flag, FlagLevel_t Level, uint16_t TimeOut){
	Result_t res = {OKE, 0};

	__IO uint32_t time = GetCounterFunction();
	while((Level == FLAG_RESET)?(*Register & Flag) : (!(*Register & Flag))){
		if(TimeOut != NO_TIMEOUT){
			if(GetCounterFunction() - time >= TimeOut) {
				res.Status  = TIMEOUT;
				return res;
			}
		}
	}
	return res;
}

/**
 * @fn Result_t wait_check_flag_in_register_timeout(volatile uint32_t*, uint32_t, FlagLevel_t, volatile uint32_t*, uint32_t, FlagLevel_t, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param RegisterCheck
 * @param FlagCheck
 * @param LevelCheck
 * @param RegisterWait
 * @param FlagWait
 * @param LevelWait
 * @param TimeOut
 * @return
 */
Result_t wait_check_flag_in_register_timeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, FlagLevel_t LevelCheck,
									  	  	 __IO uint32_t *RegisterWait, uint32_t FlagWait, FlagLevel_t LevelWait,uint16_t TimeOut){
	Result_t res = {OKE};

	__IO uint32_t time = GetCounterFunction();
	while((LevelWait == FLAG_RESET)? (*RegisterWait & FlagWait) : (!(*RegisterWait & FlagWait))){
		if((LevelCheck == FLAG_RESET)? (!(*RegisterCheck & FlagCheck)) : (*RegisterCheck & FlagCheck)) {
			res.Status = ERR;
			return res;
		}
		if(TimeOut != NO_TIMEOUT){
			if(GetCounterFunction() - time >= TimeOut) {
				res.Status  = TIMEOUT;
				return res;
			}
		}
	}

	return res;
}

/**
 * @fn void set_result(Result_t*, Status_t, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @param Status
 * @param CodeLine
 */
void set_result(Result_t *res, Status_t Status, uint32_t CodeLine){
	res -> Status = Status;
	res -> CodeLine = CodeLine;
}

/**
 * @fn void set_result_line(Result_t*, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @param line
 */
void set_result_line(Result_t *res, uint16_t line){
	res -> CodeLine = line;
}

/**
 * @fn bool result_is_err(Result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool result_is_err(Result_t *res){
	if(res -> Status == ERR) return true;
	return false;
}

/**
 * @fn bool result_is_oke(Result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool result_is_oke(Result_t *res){
	if(res -> Status == OKE) return true;
	return false;
}

/**
 * @fn bool result_is_timeout(Result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool result_is_timeout(Result_t *res){
	if(res -> Status == TIMEOUT) return true;
	return false;
}

/**
 * @fn bool result_is_nosupport(Result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool result_is_nosupport(Result_t *res){
	if(res -> Status == NOSUPPORT) return true;
	return false;
}

/**
 * @fn bool result_is_busy(Result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool result_is_busy(Result_t *res){
	if(res -> Status == BUSY) return true;
	return false;
}

/**
 * @fn bool result_is_ready(Result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool result_is_ready(Result_t *res){
	if(res -> Status == READY) return true;
	return false;
}



















