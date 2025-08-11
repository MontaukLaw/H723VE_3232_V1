#include "user_comm.h"

static uint8_t ads131_tx_data[9] = {0};
static uint8_t ads131_rx_data[9] = {0};
static uint8_t adc_result_idx = 0;
volatile uint32_t adc_mapped = 0;

//*****************************************************************************
//
//! Sends SPI byte on MOSI pin and captures MISO return byte value.
//!
//! \fn uint8_t spiSendReceiveByte(const uint8_t dataTx)
//!
//! \param const uint8_t dataTx data byte to send on MOSI pin.
//!
//! NOTE: This function is called by spiSendReceiveArrays(). If it is called
//! directly, then the /CS pin must also be directly controlled.
//!
//! \return Captured MISO response byte.
//
//*****************************************************************************
uint8_t spiSendReceiveByte(const uint8_t dataTx)
{
    /*  --- INSERT YOUR CODE HERE ---
     *  This function should send and receive single bytes over the SPI.
     *  NOTE: This function does not control the /CS pin to allow for
     *  more programming flexibility.
     */

    // SSI TX & RX
    uint8_t dataRx = 0;

    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)&dataTx, (uint8_t *)&dataRx, 1, 1000);
    // HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t *)&dataTx, (uint8_t *)&dataRx, 1);

    return dataRx;
}

//*****************************************************************************
//
//! Sends SPI byte array on MOSI pin and captures MISO data to a byte array.
//!
//! \fn void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
//!
//! \param const uint8_t dataTx[] byte array of SPI data to send on MOSI.
//!
//! \param uint8_t dataRx[] byte array of SPI data captured on MISO.
//!
//! \param uint8_t byteLength number of bytes to send & receive.
//!
//! NOTE: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************
void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
{
    /*  --- INSERT YOUR CODE HERE ---
     *
     *  This function should send and receive multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the /CS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the /CS pin high (if controlled by GPIO)
     */

    // Require that dataTx and dataRx are not NULL pointers
    assert(dataTx && dataRx);

    // Set the nCS pin LOW
    // setCS(LOW);
    ADS131_CS_GPIO_Port->BSRR = (uint32_t)ADS131_CS_Pin << 16U;
    // Send all dataTx[] bytes on MOSI, and capture all MISO bytes in dataRx[]
    // int i;
    // for (i = 0; i < byteLength; i++)
    // {
    // dataRx[i] = spiSendReceiveByte(dataTx[i]);
    // }

    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)dataTx, (uint8_t *)dataRx, byteLength, 1000);
    ADS131_CS_GPIO_Port->BSRR = ADS131_CS_Pin;
    // Set the nCS pin HIGH
    // setCS(HIGH);
}

//*****************************************************************************
//
//! Controls the state of the /CS GPIO pin.
//!
//! \fn void setCS(const bool state)
//!
//! \param state boolean indicating which state to set the /CS pin (0=low, 1=high)
//!
//! NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
//! function for the 'state' parameter value.
//!
//! \return None.
//
//*****************************************************************************
void setCS(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */

    // td(CSSC) delay
    if (state)
    {
        delay_us(2);
    }

    // LL_GPIO_WriteOutputPort(ADS131_CS_GPIO_Port, state ? ADS131_CS_Pin : 0x00); // CS拉高
    HAL_GPIO_WritePin(ADS131_CS_GPIO_Port, ADS131_CS_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // td(SCCS) delay
    if (!state)
    {
        delay_us(2);
    }
}

void delay_ms(const uint32_t delay_time_ms)
{
    HAL_Delay(delay_time_ms);
}

//*****************************************************************************
//
//! Controls the state of the nSYNC/nRESET GPIO pin.
//!
//! \fn void setSYNC_RESET(const bool state)
//!
//! \param state boolean indicating which state to set the nSYNC/nRESET pin (0=low, 1=high)
//!
//! NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
//! function for the 'state' parameter value.
//!
//! \return None.
//
//*****************************************************************************
void setSYNC_RESET(const bool state)
{
    HAL_GPIO_WritePin(ADS131_RST_GPIO_Port, ADS131_RST_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//*****************************************************************************
//
//! Toggles the "nSYNC/nRESET" pin to trigger a synchronization
//! (LOW, delay 2 us, then HIGH).
//!
//! \fn void toggleSYNC(void)
//!
//! \return None.
//
//*****************************************************************************
void toggleSYNC(void)
{
    HAL_GPIO_WritePin(ADS131_RST_GPIO_Port, ADS131_RST_Pin, GPIO_PIN_RESET);

    // nSYNC pulse width must be between 1 and 2,048 CLKIN periods
    delay_us(2);
    HAL_GPIO_WritePin(ADS131_RST_GPIO_Port, ADS131_RST_Pin, GPIO_PIN_SET);
}

//*****************************************************************************
//
//! Toggles the "nSYNC/nRESET" pin to trigger a reset
//! (LOW, delay 2 ms, then HIGH).
//!
//! \fn void toggleRESET(void)
//!
//! \return None.
//
//*****************************************************************************
void toggleRESET(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    HAL_GPIO_WritePin(ADS131_RST_GPIO_Port, ADS131_RST_Pin, GPIO_PIN_RESET);

    // Minimum /RESET pulse width (tSRLRST) equals 2,048 CLKIN periods (1 ms @ 2.048 MHz)
    delay_ms(2);

    HAL_GPIO_WritePin(ADS131_RST_GPIO_Port, ADS131_RST_Pin, GPIO_PIN_SET);

    // tREGACQ delay before communicating with the device again
    delay_us(5);

    // NOTE: The ADS131M0x's next response word should be (0xFF20 | CHANCNT).
    // A different response may be an indication that the device did not reset.

    // Update register array
    restoreRegisterDefaults();

    // Write to MODE register to enforce mode settings
    writeSingleRegister(MODE_ADDRESS, MODE_DEFAULT);
}

// 5-6us
void get_ads_131_data(void)
{
    volatile int32_t adc_raw = 0;

    memset(ads131_tx_data, 0, sizeof(ads131_tx_data)); // 清空发送数据
    memset(ads131_rx_data, 0, sizeof(ads131_rx_data)); // 清空接收数据

    HAL_GPIO_WritePin(ADS131_CS_GPIO_Port, ADS131_CS_Pin, GPIO_PIN_RESET); // 拉低CS
    HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)ads131_tx_data, (uint8_t *)ads131_rx_data, 9, 1000);
    HAL_GPIO_WritePin(ADS131_CS_GPIO_Port, ADS131_CS_Pin, GPIO_PIN_SET); // 拉高CS

    if (ads131_rx_data[0] == 0x05 && (ads131_rx_data[1] == 0x03 || ads131_rx_data[1] == 0x01))
    {
        adc_raw = ((int32_t)ads131_rx_data[3] << 16) | ((int32_t)ads131_rx_data[4] << 8) | (int32_t)ads131_rx_data[5];

        // 扩展符号位（因为是 24-bit 补码，需要手动符号扩展到 32-bit）
        if (adc_raw & 0x800000)
        {
            // 没有差分的话
            adc_raw |= 0xFF000000;
        }

        adc_mapped = (uint32_t)(adc_raw + 0x800000);
    }
    else
    {
        adc_mapped = 0;
    }
    
}
