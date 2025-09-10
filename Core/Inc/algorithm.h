#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

extern uint16_t running_init_zero;

extern float adc_zero_val[];

extern float adc_final_result[];

void check_initialized(void);

void check_state(void);

float ema_float(float y, float x, float a);

float float_absdiff(float a, float b);

void save_adc_data(void);

void trace_baseline(float adc_v, uint16_t point_idx);

void change_init_state(void);

void trace_baseline_u32(uint32_t adc_v, uint16_t point_idx);

uint32_t ema_u32(uint32_t prev, uint32_t x, uint32_t a_num, uint32_t a_den);

#endif
