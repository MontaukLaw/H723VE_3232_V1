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

#endif
