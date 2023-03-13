/*
 * iwdg.cpp
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */
#include "peripheral_enable.h"
#if ENABLE_IWDG

#include "iwdg.h"

return_t iwdg_init(uint32_t psc, uint32_t arr){
	return_t ret;

	IWDG -> KR = IWDG_KEY_ENABLE;

	IWDG -> KR = IWDG_KEY_WRITE_ACCESS_ENABLE;

	IWDG -> PR = psc;
	IWDG -> RLR = arr-1;

	ret = wait_flag_in_register_timeout(&(IWDG->SR), (IWDG_SR_RVU | IWDG_SR_PVU), FLAG_RESET, IWDG_DEFAULT_TIMEOUT);

	IWDG -> KR = IWDG_KEY_RELOAD;

	return ret;
}

return_t iwdg_refresh(void){
	IWDG -> KR = IWDG_KEY_RELOAD;

	return {OKE, 0};
}



#endif /* ENABLE_IWDG */
