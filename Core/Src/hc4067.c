#include "user_comm.h"

volatile uint16_t adc_point_idx = 0;
volatile g_point_t point = {0, 0};

void set_wave_ch_with_wave_ch_idx(uint16_t wave_ch)
{

    uint8_t wave_ch_idx = wave_ch & 0x0f; // 只取低4位
    if (wave_ch < WAVE_CH_MAX / 2)
    {
        HAL_GPIO_WritePin(HC4067_OUT1_S0_GPIO_Port, HC4067_OUT1_S0_Pin, (wave_ch_idx & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_OUT1_S1_GPIO_Port, HC4067_OUT1_S1_Pin, (wave_ch_idx & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_OUT1_S2_GPIO_Port, HC4067_OUT1_S2_Pin, (wave_ch_idx & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_OUT1_S3_GPIO_Port, HC4067_OUT1_S3_Pin, (wave_ch_idx & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(HC4067_OUT2_S0_GPIO_Port, HC4067_OUT2_S0_Pin, (wave_ch_idx & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_OUT2_S1_GPIO_Port, HC4067_OUT2_S1_Pin, (wave_ch_idx & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_OUT2_S2_GPIO_Port, HC4067_OUT2_S2_Pin, (wave_ch_idx & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_OUT2_S3_GPIO_Port, HC4067_OUT2_S3_Pin, (wave_ch_idx & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void set_adc_ch_with_adc_ch_idx(uint16_t adc_ch)
{
    uint8_t adc_ch_idx = adc_ch & 0x0f; // 只取低4位
    if (adc_ch < ADC_CH_MAX / 2)
    {
        HAL_GPIO_WritePin(HC4067_IN1_S0_GPIO_Port, HC4067_IN1_S0_Pin, (adc_ch_idx & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_IN1_S1_GPIO_Port, HC4067_IN1_S1_Pin, (adc_ch_idx & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_IN1_S2_GPIO_Port, HC4067_IN1_S2_Pin, (adc_ch_idx & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_IN1_S3_GPIO_Port, HC4067_IN1_S3_Pin, (adc_ch_idx & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(HC4067_IN2_S0_GPIO_Port, HC4067_IN2_S0_Pin, (adc_ch_idx & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_IN2_S1_GPIO_Port, HC4067_IN2_S1_Pin, (adc_ch_idx & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_IN2_S2_GPIO_Port, HC4067_IN2_S2_Pin, (adc_ch_idx & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HC4067_IN2_S3_GPIO_Port, HC4067_IN2_S3_Pin, (adc_ch_idx & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void turn_on_wave_ch(uint16_t wave_ch)
{
    // 先都关了
    HAL_GPIO_WritePin(HC4067_OUT1_EN_GPIO_Port, HC4067_OUT1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HC4067_OUT2_EN_GPIO_Port, HC4067_OUT2_EN_Pin, GPIO_PIN_SET);

    // 再打开指定的通道
    if (wave_ch < WAVE_CH_MAX / 2)
    {
        HAL_GPIO_WritePin(HC4067_OUT1_EN_GPIO_Port, HC4067_OUT1_EN_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(HC4067_OUT2_EN_GPIO_Port, HC4067_OUT2_EN_Pin, GPIO_PIN_RESET);
    }
    
}

void turn_on_adc_ch(uint16_t adc_ch)
{
    // 先都关了
    HAL_GPIO_WritePin(HC4067_IN1_EN_GPIO_Port, HC4067_IN1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HC4067_IN2_EN_GPIO_Port, HC4067_IN2_EN_Pin, GPIO_PIN_SET);

    // 再打开指定的通道
    if (adc_ch < ADC_CH_MAX / 2)
    {
        HAL_GPIO_WritePin(HC4067_IN1_EN_GPIO_Port, HC4067_IN1_EN_Pin, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(HC4067_IN2_EN_GPIO_Port, HC4067_IN2_EN_Pin, GPIO_PIN_RESET);
    }
}

void turn_all_hc4067_off(void)
{
    // 进出全关
    HAL_GPIO_WritePin(HC4067_OUT1_EN_GPIO_Port, HC4067_OUT1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HC4067_OUT2_EN_GPIO_Port, HC4067_OUT2_EN_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(HC4067_IN1_EN_GPIO_Port, HC4067_IN1_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HC4067_IN2_EN_GPIO_Port, HC4067_IN2_EN_Pin, GPIO_PIN_SET);
}
