#ifndef __APP_H_
#define __APP_H_

#include "user_comm.h"

void main_task_uart_test(void);

void main_adc_task(void);

void delay_init(void);

void delay_us(uint32_t nus);

void dac_test(void);

void main_task_peroid_test(void);

void main_hc4067_test(void);

void main_task(void);

void global_ver_init(void);

extern float adc131_data_buf[];

extern __IO uint8_t uart1_busy;

extern __IO uint8_t time_to_calculate;

extern __IO uint16_t point_idx;

// extern g_ver global_ver;

#endif /* __APP_H_ */
