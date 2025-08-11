#include "user_comm.h"

__attribute__((section("dma_buffer"), aligned(32))) 
float adc131_data_buf[TOTAL_POINT_NUMBER] = {0};

__attribute__((section("dma_buffer"), aligned(32)))
uint16_t adc_data_buf[ADC_BUF_SIZE] = {0};

__attribute__((section("dma_buffer"), aligned(32)))
uint8_t uart_tx_buf[UART_TX_BUF_SIZE] = {0};

__IO uint8_t adc_busy = 0;
__IO static uint32_t fac_us = 0;

uint32_t ads_adc_result[PERIOD_FLAGS_MAX];
__IO static uint8_t peroid_flag = 0;
__IO static uint8_t time_to_send = 0;
__IO static uint8_t time_create_wave = 0;

__IO uint8_t time_to_calculate = 0;
__IO uint8_t uart1_busy = 0;

void delay_init(void)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); // SysTick频率为HCLK
    fac_us = 550;                                        // 不论是否使用OS,fac_us都需要使用
}

void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD; // LOAD的值
    ticks = nus * fac_us;            // 需要的节拍数
    told = SysTick->VAL;             // 刚进入时的计数器值
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow; // 这里注意一下SYSTICK是一个递减的计数器就可以了.
            else
                tcnt += reload - tnow + told;
            told = tnow;
            if (tcnt >= ticks)
                break; // 时间超过/等于要延迟的时间,则退出.
        }
    };
}

void main_task_uart_test(void)
{
    uart_tx_buf[0]++;
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, sizeof(uint8_t) * 2); // 发送数据
    HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

void uart_send_data(void)
{
    HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, sizeof(uint8_t) * 2);
}

void dac_test(void)
{
    start_dac_dma();
    // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);
    // HAL_Delay(10);
    delay_ms(10);
    // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);
}

void main_adc_task(void)
{
    if (adc_busy)
    {
        return;
    }
    HAL_Delay(100);
    __IO uint16_t adc_value = 0;
    uint32_t adc_total = 0;
    uint16_t i = 0;
    for (i = 0; i < ADC_BUF_SIZE; i++)
    {
        adc_total = adc_total + adc_data_buf[i];
    }
    adc_value = adc_total / ADC_BUF_SIZE; // 计算平均值

    uart_tx_buf[0] = (adc_value >> 8) & 0xff; // 高字节
    uart_tx_buf[1] = adc_value & 0xff;        // 低字节

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data_buf, ADC_BUF_SIZE); // 启动ADC DMA传输
    uart_send_data();
    // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);

    adc_busy = 1;

    // HAL_Delay(1000);
    // HAL_ADC_Stop_DMA(&hadc1);
}

// adc callback
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_busy = 0;
        // HAL_GPIO_TogglePin(TEST_PORT_GPIO_Port, TEST_PORT_Pin); // Toggle an LED or do something with the ADC data
        // HAL_GPIO_TogglePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin);
        // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);
    }
}

// uart tx call back
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_busy = 0;
        // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);
    }
}

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if (GPIO_Pin == GPIO_PIN_8)
//     {
//         HAL_GPIO_TogglePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin);
//         ads131_data_ready = 1; // 设置数据准备好标志
//     }
// }

void get_data_process(void)
{
    if (ads131_data_ready == 0)
    {
        return;
    }
    HAL_GPIO_TogglePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin);
    // 先获取数据
    get_ads_131_data();
    ads_adc_result[peroid_flag] = adc_mapped;
    peroid_flag++;
    if (peroid_flag >= 10)
    {

        time_to_send = 1;     // 设置发送标志
        time_create_wave = 1; // 设置波形创建标志

        peroid_flag = 0;
    }

    ads131_data_ready = 0;
}

void send_data_process(void)
{
    uint16_t i = 0;
    static uint16_t counter = 0;
    if (time_to_send == 0)
    {
        return;
    }
    for (i = 0; i < PERIOD_FLAGS_MAX; i++)
    {
        uart_tx_buf[i * 4 + 0] = i;
        // uart_tx_buf[i * 4 + 1] = (ads_adc_result[i] >> 24) & 0xff; // 高字节
        uart_tx_buf[i * 4 + 1] = (ads_adc_result[i] >> 16) & 0xff; // 次高字节
        uart_tx_buf[i * 4 + 2] = (ads_adc_result[i] >> 8) & 0xff;  // 次低字节
        uart_tx_buf[i * 4 + 3] = ads_adc_result[i] & 0xff;         // 低字节
    }
    counter++;
    if (counter > 100)
    {
        counter = 0;
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buf, sizeof(uint8_t) * PERIOD_FLAGS_MAX * 4); // 发送数据 40
    }
    time_to_send = 0;
}

void wave_create_process(void)
{

    if (time_create_wave == 0)
    {
        return;
    }
    start_dac_dma();
    time_create_wave = 0;
}

void main_task_peroid_test(void)
{

    wave_create_process();

    get_data_process();

    send_data_process();

    // uint16_t i = 0;
    // if (ads131_data_ready == 0)
    // {
    //     return;
    // }

    // if (peroid_flag == 0)
    // {
    //     start_dac_dma();
    // }

    // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);

    // if (uart1_busy)
    // {
    //     HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);
    //     ads131_data_ready = 0; // 清除数据准备好标志
    //     return;
    // }

    // ads_adc_result[peroid_flag] = adc_mapped; // 保存当前的ADC值

    // peroid_flag++;
    // if (peroid_flag >= 10)
    // {

    //     uart1_busy = 1;
    //     peroid_flag = 0;

    //     delay_ms(10);
    // }

    // ads131_data_ready = 0; // 清除数据准备好标志
    // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);
}

void main_hc4067_test(void)
{
    static uint8_t wave_ch = 0;
    static uint8_t adc_ch = 0;

    for (wave_ch = 0; wave_ch < WAVE_CH_MAX; wave_ch++)
    {
        // start_dac_dma();
        set_wave_ch_with_wave_ch_idx(wave_ch);
        // turn_on_wave_ch(wave_ch);
        // delay_us(15);
        // turn_all_hc4067_off();
        for (adc_ch = 0; adc_ch < ADC_CH_MAX; adc_ch++)
        {

            // 打开wave输出
            start_dac_dma();
            set_adc_ch_with_adc_ch_idx(adc_ch);

            turn_on_wave_ch(wave_ch);
            turn_on_adc_ch(adc_ch);
            delay_us(15); // 等待一段时间
            turn_all_hc4067_off();
        }
    }
}

uint16_t change_hc4067_ch(void)
{
    static uint16_t wave_ch = 0;
    static uint16_t adc_ch = 0;
    set_wave_ch_with_wave_ch_idx(wave_ch);
    set_adc_ch_with_adc_ch_idx(adc_ch);
    turn_on_wave_ch(wave_ch);
    turn_on_adc_ch(adc_ch);

    adc_ch++;
    if (adc_ch >= ADC_CH_MAX)
    {
        adc_ch = 0;
        wave_ch++;
        if (wave_ch >= WAVE_CH_MAX)
        {
            wave_ch = 0; // 重置波形通道
        }
    }

    return wave_ch * ADC_CH_MAX + adc_ch; // 返回当前的通道组合
}

uint8_t count_peroid(uint8_t *peroid_counter)
{
    *peroid_counter = *peroid_counter + 1;
    if (*peroid_counter >= PERIOD_FLAGS_MAX)
    {
        *peroid_counter = 0;
    }

    return *peroid_counter;
}

void save_data(void)
{
    static uint16_t point_idx = 0;

    adc131_data_buf[point_idx] = (float)adc_mapped / ADC_SCALE;
    point_idx++;
    if (point_idx >= TOTAL_POINT_NUMBER)
    {
        // 指示周期开始
        HAL_GPIO_TogglePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin);
        time_to_calculate = 1;
        point_idx = 0; // 重置索引
    }
}

uint8_t calculate_data(void)
{
    if (time_to_calculate == 0)
        return 0;

    send_ads131_val_to_master();

    time_to_calculate = 0;
    return 1;
}

// 16us*1024 = 16.384ms*3 = 49.152ms
void main_task(void)
{

    comm_handler();

    static uint8_t peroid_counter = 0;

    if (if_getting_data() == 0)
    {
        return;
    }

    if (ads131_data_ready == 0)
    {
        return;
    }

    if (calculate_data())
    {

        // 先获取数据, 无论哪个周期, 都要获取数据, 避免ads131出错
        // get_ads_131_data();
        return;
    }

    if (peroid_counter == 0)
    {
        // peroid 0 切换输入输出通道
        change_hc4067_ch();
        start_dac_dma();
    }
    if (peroid_counter == 0 || peroid_counter == 1)
    {
        // start_dac_dma();
    }

    // 先获取数据, 无论哪个周期, 都要获取数据
    // get_ads_131_data();

    // step 3 获取数据
    // 7us是极限了, 再多会有问题.
    delay_us(7); // delay以获取最佳线性度

    peroid_counter++;
    if (peroid_counter >= PERIOD_FLAGS_MAX)
    {
        save_data();
        peroid_counter = 0; // 重置周期计数器
        // 关门
        turn_all_hc4067_off();
    }

    ads131_data_ready = 0;
}
