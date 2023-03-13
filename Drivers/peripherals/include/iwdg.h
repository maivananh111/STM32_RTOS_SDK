/*
 * iwdg.h
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */

#ifndef IWDG_H_
#define IWDG_H_

#include "peripheral_enable.h"
#if ENABLE_IWDG

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx.h"
#include "status.h"
#include "sdkconfig.h"
#include "stm_log.h"

#define IWDG_KEY_RELOAD                 0x0000AAAAu  /* IWDG Reload Counter Enable   */
#define IWDG_KEY_ENABLE                 0x0000CCCCu
#define IWDG_KEY_WRITE_ACCESS_ENABLE    0x00005555u
#define IWDG_KEY_WRITE_ACCESS_DISABLE   0x00000000u
#define IWDG_DEFAULT_TIMEOUT            (((6UL*256UL*1000UL) / LSI_VALUE) + ((LSI_STARTUP_TIME/1000UL)+1UL))

#define IWDG_PRESCALER_4                0x00000000U
#define IWDG_PRESCALER_8                IWDG_PR_PR_0
#define IWDG_PRESCALER_16               IWDG_PR_PR_1
#define IWDG_PRESCALER_32               (IWDG_PR_PR_1 | IWDG_PR_PR_0)
#define IWDG_PRESCALER_64               IWDG_PR_PR_2
#define IWDG_PRESCALER_128              (IWDG_PR_PR_2 | IWDG_PR_PR_0)
#define IWDG_PRESCALER_256              (IWDG_PR_PR_2 | IWDG_PR_PR_1)

return_t iwdg_init(uint32_t psc, uint32_t arr);

return_t iwdg_refresh(void);


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_IWDG */

#endif /* IWDG_H_ */
