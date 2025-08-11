#ifndef __APP_H_
#define __APP_H_

void main_task_uart_test(void);

void main_adc_task(void);

void delay_init(void);

void delay_us(uint32_t nus);

void dac_test(void);

void main_task_peroid_test(void);

void main_hc4067_test(void);

void main_task(void);

extern float adc131_data_buf[];
extern __IO uint8_t uart1_busy;

#endif /* __APP_H_ */
