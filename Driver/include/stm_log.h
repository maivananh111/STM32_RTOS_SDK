/*
 * stm_log.h
 *
 *  Created on: 28 thg 7, 2022
 *      Author: anh
 */

#ifndef STM_LOG_H_
#define STM_LOG_H_


#include "stdio.h"
#include "stdarg.h"
#include "status.h"

#define LOG_TO_ESP
#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	SIMP_BLACK = 0,
	SIMP_RED,
	SIMP_GREEN,
	SIMP_YELLOW,
	SIMP_BLUE,
	SIMP_PURPLE,
	SIMP_CYAN,
	SIMP_WHITE,

	BOLD_BLACK = 8,
	BOLD_RED,
	BOLD_GREEN,
	BOLD_YELLOW,
	BOLD_BLUE,
	BOLD_PURPLE,
	BOLD_CYAN,
	BOLD_WHITE,

	ITALIC_BLACK = 16,
	ITALIC_RED,
	ITALIC_GREEN,
	ITALIC_YELLOW,
	ITALIC_BLUE,
	ITALIC_PURPLE,
	ITALIC_CYAN,
	ITALIC_WHITE,

	BCKGRN_BLACK = 24,
	BCKGRN_RED,
	BCKGRN_GREEN,
	BCKGRN_YELLOW,
	BCKGRN_BLUE,
	BCKGRN_PURPLE,
	BCKGRN_CYAN,
	BCKGRN_WHITE,
} log_type_t;

void stm_log_init(void (*PrintString_Function)(char*));

void stm_set_log(char *func, log_type_t log_type);

void STM_LOG(log_type_t log_type, const char *tag, const char *format, ...);

void STM_LOGI(const char *tag,  const char *format, ...);
void STM_LOGW(const char *tag,  const char *format, ...);
void STM_LOGE(const char *tag,  const char *format, ...);
void STM_LOGD(const char *tag,  const char *format, ...);
void STM_LOGP(const char *tag,  const char *format, ...);
void STM_LOGM(const char *tag,  const char *format, ...);
void STM_LOGR(const char *tag,  const char *format, ...);

void STM_LOG_RES(Result_t res);
void STM_LOG_MEM(void);

#ifdef __cplusplus
}
#endif


#endif /* STM_LOG_H_ */
