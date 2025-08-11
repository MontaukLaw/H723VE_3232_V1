#include "user_comm.h"

__attribute__((section("dma_buffer"), aligned(32))) // 用于存储点数据, 保存在CPU可以访问的区域
uint16_t sine_val[DAC_LEN];

__attribute__((section("dma_buffer"), aligned(32))) // 用于存储点数据, 保存在CPU可以访问的区域
uint16_t dac_buf[4] = {0, 1000, 1000, 1000};

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
            ///sine_val[idx] = DAC_MAX_VALUE;
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
