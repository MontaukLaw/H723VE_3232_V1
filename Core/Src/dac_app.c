#include "user_comm.h"

__attribute__((section("dma_buffer"), aligned(32))) // 用于存储点数据, 保存在CPU可以访问的区域
uint16_t sine_val[DAC_LEN];

__attribute__((section("dma_buffer"), aligned(32))) // 用于存储点数据, 保存在CPU可以访问的区域
uint16_t dac_buf[4] = {0, 1000, 1000, 1000};

void get_straight_wave(void)
{
    uint16_t i;
    uint16_t j;
    uint16_t idx = 0;

    for (j = 0; j < DAC_LEN; j++)
    {
        sine_val[j] = DAC_MAX_VALUE;
    }

    sine_val[DAC_LEN - 1] = 0; // 清零作为最后的电平
}

// 14uS周期
// 71k
void get_sine_wave(void)
{
    for (uint32_t idx = 0; idx < DAC_LEN; idx++)
    {
        // 整个缓冲区只走 0~2π 一圈
        double w = (double)idx * 2.0 * PI / (double)DAC_LEN;
        double s = (sin(w) * 0.5 + 0.5) * (DAC_MAX_VALUE - HEADROOM * 2) + HEADROOM;
        uint32_t y = (uint32_t)lrint(s) + DAC_FLAT_VAL;
        if (y > DAC_MAX_VALUE)
            y = DAC_MAX_VALUE;
        sine_val[idx] = (uint16_t)y;
    }
}

void get_sine_wave_(void)
{
    uint16_t i;
    uint16_t j;
    uint16_t idx = 0;

    for (j = 0; j < MAX_SAMPLES_TIMES; j++)
    {
        for (i = 0; i < MAX_SAPMLES; i++)
        {
            double w = (double)idx * 2.0 * PI / (double)MAX_SAPMLES;
            double s = (sin(w) * 0.5 + 0.5) * (DAC_MAX_VALUE - HEADROOM * 2) + HEADROOM;
            uint32_t y = (uint32_t)lrint(s);
            if (y > DAC_MAX_VALUE)
                y = DAC_MAX_VALUE;

            sine_val[idx] = (uint16_t)y;
            idx++;
        }
    }
    sine_val[DAC_LEN - 1] = DAC_MAX_VALUE / 2;
}

void get_half_sine_wave(void)
{
    uint16_t i;
    uint16_t j;
    uint16_t idx = 0;

    for (j = 0; j < MAX_SAMPLES_TIMES; j++)
    {
        for (i = 0; i < MAX_SAPMLES; i++)
        {
            // 让角度从 0 ~ π 线性变化
            double w = (double)i * PI / MAX_SAPMLES;
            // +1 是为了把 sin(w) 的范围 [-1, 1] 提升到 [0, 2]

            sine_val[idx] = (sin(w)) * DAC_MAX_VALUE;
            // sine_val[idx] = (cos(w)) * DAC_MAX_VALUE;
            // sine_val[idx] = DAC_MAX_VALUE;
            // sine_val[idx] = 0;
            // sine_val[idx] = 256;
            // sine_val[idx] = 4095;
            idx++;
        }
    }

    sine_val[DAC_LEN - 1] = 0; // 清零作为最后的电平
}

void start_dac_dma(void)
{
    HAL_TIM_Base_Stop(&htim7);
    HAL_TIM_Base_Start(&htim7);

    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sine_val, DAC_LEN, DAC_ALIGN_12B_R);
}

// dac callback
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{

    if (hdac->Instance == DAC1)
    {
        // HAL_GPIO_TogglePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin);
        // HAL_GPIO_TogglePin(FOR_TEST2_GPIO_Port, FOR_TEST2_Pin);
        // HAL_GPIO_WritePin(TEST2_GPIO_Port, FOR_TEST2_Pin, GPIO_PIN_RESET);
    }
}
