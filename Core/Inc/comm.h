#ifndef _COMM_H_
#define _COMM_H_

void start_uart_rx(void);

void comm_handler(void);

void send_ads131_val_to_master(void);

uint8_t if_getting_data(void);

void send_ads131_val_to_udp(void);

extern __IO uint8_t got_rx;

#endif
