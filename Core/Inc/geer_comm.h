#ifndef _GEER_COMM_H_
#define _GEER_COMM_H_

uint16_t checksum_crc16(uint8_t const *ptr, uint16_t len);

void geer_comm_handler(void);

void uart_test(void);

void data_upload_handle(void);

#endif
