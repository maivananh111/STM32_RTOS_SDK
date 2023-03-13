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
 * @fn status_t check_flag_in_register(volatile uint32_t*, uint32_t, flaglevel_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param StatusCheck
 * @return
 */
status_t check_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t StatusCheck){
	if((StatusCheck == FLAG_SET)? ((*Register & Flag) != 0U) : ((*Register & Flag) == 0U)) return OKE;
	return ERR;
}

/**
 * @fn void wait_flag_in_register(volatile uint32_t*, uint32_t, flaglevel_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param Level
 */
void wait_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level){
	while((Level == FLAG_RESET)?((*Register & Flag)) : (!(*Register & Flag)));
}

/**
 * @fn return_t wait_flag_in_register_timeout(volatile uint32_t*, uint32_t, flaglevel_t, uint16_t)
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
return_t wait_flag_in_register_timeout(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level, uint16_t TimeOut){
	return_t res = {OKE, 0};

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
 * @fn return_t wait_check_flag_in_register_timeout(volatile uint32_t*, uint32_t, flaglevel_t, volatile uint32_t*, uint32_t, flaglevel_t, uint16_t)
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
return_t wait_check_flag_in_register_timeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, flaglevel_t LevelCheck,
									  	  	 __IO uint32_t *RegisterWait, uint32_t FlagWait, flaglevel_t LevelWait,uint16_t TimeOut){
	return_t res = {OKE};

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
 * @fn void set_return(return_t*, status_t, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @param Status
 * @param CodeLine
 */
void set_return(return_t *res, status_t Status, uint32_t CodeLine){
	res -> Status = Status;
	res -> Line = CodeLine;
}

/**
 * @fn void set_return_line(return_t*, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @param line
 */
void set_return_line(return_t *res, uint16_t line){
	res -> Line = line;
}

/**
 * @fn bool is_err(return_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_err(return_t *res){
	if(res -> Status == ERR) return true;
	return false;
}

/**
 * @fn bool is_oke(return_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_oke(return_t *res){
	if(res -> Status == OKE) return true;
	return false;
}

/**
 * @fn bool is_timeout(return_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_timeout(return_t *res){
	if(res -> Status == TIMEOUT) return true;
	return false;
}

/**
 * @fn bool is_unsupported(return_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_unsupported(return_t *res){
	if(res -> Status == UNSUPPORTED) return true;
	return false;
}

/**
 * @fn bool is_busy(return_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_busy(return_t *res){
	if(res -> Status == BUSY) return true;
	return false;
}

/**
 * @fn bool is_ready(return_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_ready(return_t *res){
	if(res -> Status == READY) return true;
	return false;
}

/**
 * @fn bool is_unavailable(return_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_notavailable(return_t *res){
	if(res -> Status == NOTAVAILABLE) return true;
	return false;
}


















