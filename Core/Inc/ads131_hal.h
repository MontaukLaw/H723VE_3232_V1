#ifndef _ADS_HAL_131_H_
#define _ADS_HAL_131_H_

void InitADC(void);
void delay_ms(const uint32_t delay_time_ms);
// void delay_us(const uint32_t delay_time_us);
void setCS(const bool state);
void setSYNC_RESET(const bool state);
void toggleSYNC(void);
void toggleRESET(void);
void spiSendReceiveArrays(const uint8_t DataTx[], uint8_t DataRx[], const uint8_t byteLength);
uint8_t spiSendReceiveByte(const uint8_t dataTx);
bool waitForDRDYinterrupt(const uint32_t timeout_ms);

void spi_3_test(void);

void get_ads_131_data(void);

extern volatile uint32_t adc_mapped;

#endif
