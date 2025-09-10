#ifndef __LOGGER_H_
#define __LOGGER_H_

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

void U5Log_Init(UART_HandleTypeDef *huart5);

/* 直接往日志队列写入字节流（非阻塞）。返回写入的字节数（可能小于len表示队列满）。 */
size_t U5Log_Write(const uint8_t *data, size_t len);

/* 可选：把 '\n' 自动转为 "\r\n" */
void U5Log_SetAutoCRLF(uint8_t enable);

void send_data_u5(void);

int myprintf(const char *format, ...);

void ads131_frame_counter(void);

#define printf myprintf

#ifdef __cplusplus
}
#endif

#endif

