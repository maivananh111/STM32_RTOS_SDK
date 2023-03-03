/*
 * stm_log.cpp
 *
 *  Created on: 28 thg 7, 2022
 *      Author: A315-56
 */

#include "stm_log.h"

#include "rcc.h"
#include "system.h"
#include "systick.h"
#include "sdkconfig.h"

#include "string.h"
#include "stdlib.h"
#include "stdio.h"

static log_type_t logi = SIMP_GREEN;	// Information.
static log_type_t logw = SIMP_YELLOW;   // Warning.
static log_type_t loge = SIMP_RED;		// Error.
static log_type_t logd = SIMP_BLUE;		// Debug.
static log_type_t logm = SIMP_WHITE;	// Memory.
static log_type_t logp = SIMP_CYAN;	    // Parameter.
static log_type_t logr = SIMP_PURPLE;	// Result.


static const char *Result_Str[6] = {
	"ERR",
	"OKE",
	"TIMEOUT",
	"NOTSUPPORT",
	"BUSY",
	"READY"
};
void (*LOG)(char *str);
static const char *COLOR_END = "\033[0m";
static const char *LOG_COLOR[] = {
	"\033[0;30m",
	"\033[0;31m",
	"\033[0;32m",
	"\033[0;33m",
	"\033[0;34m",
	"\033[0;35m",
	"\033[0;36m",
	"\033[0;37m",

	// Bold
	"\033[1;30m",
	"\033[1;31m",
	"\033[1;32m",
	"\033[1;33m",
	"\033[1;34m",
	"\033[1;35m",
	"\033[1;36m",
	"\033[1;37m",

	// Italic
	"\033[4;30m",
	"\033[4;31m",
	"\033[4;32m",
	"\033[4;33m",
	"\033[4;34m",
	"\033[4;35m",
	"\033[4;36m",
	"\033[4;37m",

	// Background
	"\033[40m",
	"\033[41m",
	"\033[42m",
	"\033[43m",
	"\033[44m",
	"\033[45m",
	"\033[46m",
	"\033[47m",
};

/**
 * @fn void stm_log_init(void(*)(char*))
 * @brief
 *
 * @pre
 * @post
 * @param PrintString_Function
 */
void stm_log_init(void (*PrintString_Function)(char*)){
	LOG = PrintString_Function;
}

/**
 * @fn void stm_set_log(char*, log_type_t)
 * @brief
 *
 * @pre
 * @post
 * @param func
 * @param log_type
 */
void stm_set_log(char *func, log_type_t log_type){
	if	   (strcmp(func, (char *)"STM_LOGI") == 0) logi = log_type;
	else if(strcmp(func, (char *)"STM_LOGW") == 0) logw = log_type;
	else if(strcmp(func, (char *)"STM_LOGE") == 0) loge = log_type;
	else if(strcmp(func, (char *)"STM_LOGD") == 0) logd = log_type;
	else if(strcmp(func, (char *)"STM_LOGM") == 0) logm = log_type;
	else if(strcmp(func, (char *)"STM_LOGP") == 0) logp = log_type;
	else if(strcmp(func, (char *)"STM_LOGR") == 0) logr = log_type;
	else STM_LOGE("Parameter Error", "Unknown function %s.", func);
}

/**
 * @fn void STM_LOG(log_type_t, const char*, const char*, ...)
 * @brief
 *
 * @pre
 * @post
 * @param log_type
 * @param tag
 * @param format
 */

void STM_LOG(log_type_t log_type, const char *tag, const char *format, ...){
#if LOG_TIME
	uint32_t time = get_tick();
#endif
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[log_type]);
#if LOG_TIME
	uint16_t totallen = (color_start_length + 14 + strlen(tag) + 2 + length + 4) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s[%10lu] %s: %s%s\n\r", LOG_COLOR[log_type], time, tag, Temp_buffer, COLOR_END);
#else
	uint16_t totallen = (color_start_length + strlen(tag) + 2 + length + 4 + 2) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s: %s%s\n\r", LOG_COLOR[log_type], tag, Temp_buffer, COLOR_END);
#endif
	Output_buffer[totallen] = '\0';

	LOG(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

/**
 * @fn void STM_LOGI(const char*, const char*, ...)
 * @brief
 *
 * @pre
 * @post
 * @param tag
 * @param format
 */
void STM_LOGI(const char *tag,  const char *format, ...){
#if LOG_TIME
	uint32_t time = get_tick();
#endif
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[logi]);
#if LOG_TIME
	uint16_t totallen = (color_start_length + 7 + 15 + strlen(tag) + 2 + length + 4) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen + 1);
	sprintf(Output_buffer, "%s%s [%lu] %s: %s%s\n\r", LOG_COLOR[logi], "INFOR ", time, tag, Temp_buffer, COLOR_END);
#else
	uint16_t totallen = (color_start_length + 7 + strlen(tag) + 2 + length + 4 + 2) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s %s: %s%s\n\r", LOG_COLOR[logi], "INFOR ", tag, Temp_buffer, COLOR_END);
#endif
	Output_buffer[totallen] = '\0';

	LOG(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

/**
 * @fn void STM_LOGW(const char*, const char*, ...)
 * @brief
 *
 * @pre
 * @post
 * @param tag
 * @param format
 */
void STM_LOGW(const char *tag,  const char *format, ...){
#if LOG_TIME
	uint32_t time = get_tick();
#endif
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[logw]);
#if LOG_TIME
	uint16_t totallen = (color_start_length + 7 + 15 + strlen(tag) + 2 + length + 4) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s [%lu] %s: %s%s\n\r", LOG_COLOR[logw], "WARN  ", time, tag, Temp_buffer, COLOR_END);
#else
	uint16_t totallen = (color_start_length + 7 + strlen(tag) + 2 + length + 4 + 2) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s %s: %s%s\n\r", LOG_COLOR[logw], "WARN  ", tag, Temp_buffer, COLOR_END);
#endif
	Output_buffer[totallen] = '\0';

	LOG(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

/**
 * @fn void STM_LOGE(const char*, const char*, ...)
 * @brief
 *
 * @pre
 * @post
 * @param tag
 * @param format
 */
void STM_LOGE(const char *tag,  const char *format, ...){
#if LOG_TIME
	uint32_t time = get_tick();
#endif
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[loge]);
#if LOG_TIME
	uint16_t totallen = (color_start_length + 7 + 15 + strlen(tag) + 2 + length + 4) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s [%lu] %s: %s%s\n\r", LOG_COLOR[loge], "ERROR ", time, tag, Temp_buffer, COLOR_END);
#else
	uint16_t totallen = (color_start_length + 7 + strlen(tag) + 2 + length + 4 + 2) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s %s: %s%s\n\r", LOG_COLOR[loge], "ERROR ", tag, Temp_buffer, COLOR_END);
#endif
	Output_buffer[totallen] = '\0';

	LOG(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

/**
 * @fn void STM_LOGD(const char*, const char*, ...)
 * @brief
 *
 * @pre
 * @post
 * @param tag
 * @param format
 */
void STM_LOGD(const char *tag,  const char *format, ...){
#if LOG_TIME
	uint32_t time = get_tick();
#endif
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[logd]);
#if LOG_TIME
	uint16_t totallen = (color_start_length + 7 + 15 + strlen(tag) + 2 + length + 4) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s [%lu] %s: %s%s\n\r", LOG_COLOR[logd], "DEBUG ", time, tag, Temp_buffer, COLOR_END);
#else
	uint16_t totallen = (color_start_length + 7 + strlen(tag) + 2 + length + 4 + 2) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s %s: %s%s\n\r", LOG_COLOR[logd], "DEBUG ", tag, Temp_buffer, COLOR_END);
#endif
	Output_buffer[totallen] = '\0';

	LOG(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

/**
 * @fn void STM_LOGM(const char*, const char*, ...)
 * @brief
 *
 * @pre
 * @post
 * @param tag
 * @param format
 */
void STM_LOGM(const char *tag,  const char *format, ...){
#if LOG_TIME
	uint32_t time = get_tick();
#endif
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[logm]);
#if LOG_TIME
	uint16_t totallen = (color_start_length + 7 + 15 + strlen(tag) + 2 + length + 4) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s [%lu] %s: %s%s\n\r", LOG_COLOR[logm], "MEMORY", time, tag, Temp_buffer, COLOR_END);
#else
	uint16_t totallen = (color_start_length + 7 + strlen(tag) + 2 + length + 4 + 2) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s %s: %s%s\n\r", LOG_COLOR[logm], "MEMORY", tag, Temp_buffer, COLOR_END);
#endif
	Output_buffer[totallen] = '\0';

	LOG(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

/**
 * @fn void STM_LOGP(const char*, const char*, ...)
 * @brief
 *
 * @pre
 * @post
 * @param tag
 * @param format
 */
void STM_LOGP(const char *tag,  const char *format, ...){
#if LOG_TIME
	uint32_t time = get_tick();
#endif
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[logp]);
#if LOG_TIME
	uint16_t totallen = (color_start_length + 7 + 15 + strlen(tag) + 2 + length + 4) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s [%lu] %s: %s%s\n\r", LOG_COLOR[logp], "PARAM ", time, tag, Temp_buffer, COLOR_END);
#else
	uint16_t totallen = (color_start_length + 7 + strlen(tag) + 2 + length + 4 + 2) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s %s: %s%s\n\r", LOG_COLOR[logp], "PARAM ", tag, Temp_buffer, COLOR_END);
#endif
	Output_buffer[totallen] = '\0';

	LOG(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}

/**
 * @fn void STM_LOGR(const char*, const char*, ...)
 * @brief
 *
 * @pre
 * @post
 * @param tag
 * @param format
 */
void STM_LOGR(const char *tag,  const char *format, ...){
#if LOG_TIME
	uint32_t time = get_tick();
#endif
	char *Temp_buffer = NULL;
	va_list args;
	va_start(args, format);

	uint16_t length = vsnprintf(Temp_buffer, 0, format, args);
	Temp_buffer = (char *)malloc(length * sizeof(char) + 1);
	vsprintf(Temp_buffer, format, args);
	va_end(args);

	uint8_t color_start_length = strlen(LOG_COLOR[logr]);
#if LOG_TIME
	uint16_t totallen = (color_start_length + 7 + 15 + strlen(tag) + 2 + length + 4) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s [%lu] %s: %s%s\n\r", LOG_COLOR[logr], "RESULT", time, tag, Temp_buffer, COLOR_END);
#else
	uint16_t totallen = (color_start_length + 7 + strlen(tag) + 2 + length + 4 + 2) * sizeof(char);
	char *Output_buffer = (char *)malloc(totallen+1);
	sprintf(Output_buffer, "%s%s %s: %s%s\n\r", LOG_COLOR[logr], "RESULT", tag, Temp_buffer, COLOR_END);
#endif
	Output_buffer[totallen] = '\0';

	LOG(Output_buffer);
	free(Temp_buffer);
	free(Output_buffer);
}


/**
 * @fn void STM_LOG_RES(return_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 */
void STM_LOG_RES(return_t res){
	STM_LOGR("RESULT", "Return %s, time = %d[%s -> %s -> %d]", Result_Str[res.Status], res.Line);
}

/**
 * @fn void STM_LOG_MEM(void)
 * @brief
 *
 * @pre
 * @post
 */
void STM_LOG_MEM(void){
	memory_info_t mem = get_memory_info();
	STM_LOGM("USED", "heap_ram_used %lu, prog_ram_used %lu, stack_ram_used %lu.", mem.heap_ram_used, mem.prog_ram_used, mem.stack_ram_used);
	STM_LOGM("FREE", "total_free_ram %lu, free_ram %lu.", mem.total_free_ram, mem.free_ram);
}








