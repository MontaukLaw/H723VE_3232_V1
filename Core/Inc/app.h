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

void main_wave_test(void);

void change_hc4067_ch(void);

uint8_t calculate_data(void);

void save_data(void);

void change_point_idx(void);

void main_hc4067_test_1ch(void);

void save_data_simple(uint16_t point_id);

void save_data_directly(uint16_t point_id);

void cp_all_data_to_ram_u16(void);

void cp_all_float_data_to_ram_u16(void);

extern float adc131_data_buf[];

extern __IO uint8_t uart1_busy;

extern __IO uint8_t time_to_calculate;

extern __IO uint16_t point_idx;

extern __IO uint32_t adc_final_result_u32[];

extern __IO uint16_t adc_result_16bit[];

// extern g_ver global_ver;

#endif /* __APP_H_ */
