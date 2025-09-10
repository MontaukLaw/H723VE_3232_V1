#include "user_comm.h"

g_ver global_ver = {0};
__IO uint16_t point_idx = 0;

__attribute__((section("dma_buffer"), aligned(32))) float adc131_data_buf[TOTAL_POINT_NUMBER] = {0};

__attribute__((section("dma_buffer"), aligned(32)))
uint16_t adc_data_buf[ADC_BUF_SIZE] = {0};

__attribute__((section("dma_buffer"), aligned(32)))
uint8_t uart_tx_buf[UART_TX_BUF_SIZE] = {0};

__IO uint16_t adc_result_16bit[TOTAL_POINT_NUMBER] = {0};
__IO uint16_t adc_result_16bit_buf[TOTAL_POINT_NUMBER] = {0};

__IO uint8_t adc_busy = 0;
__IO static uint32_t fac_us = 0;

uint32_t ads_adc_result[PERIOD_FLAGS_MAX];
__IO static uint8_t peroid_flag = 0;
__IO static uint8_t time_to_send = 0;
__IO static uint8_t time_create_wave = 0;

__IO uint8_t uart1_busy = 0;

volatile float last_iir_adc_val[TOTAL_SENSOR_NUMBER] = {0};
volatile uint32_t last_iir_adc_val_u32[TOTAL_SENSOR_NUMBER] = {0};
__IO uint32_t adc_final_result_u32[TOTAL_SENSOR_NUMBER] = {0};

void global_ver_init(void)
{
    global_ver.initialized = false;
    global_ver.state = ST_IDLE;
    memset(global_ver.baseline, 0, sizeof(global_ver.baseline));
    memset(global_ver.base_noise, 0, sizeof(global_ver.base_noise));
}

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
    else if (huart->Instance == UART5)
    {
        send_data_u5();
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

void main_hc4067_test_1ch(void)
{
    static uint8_t wave_ch = 0;
    static uint8_t adc_ch = 0;
    HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);

    start_dac_dma();
    set_wave_ch_with_wave_ch_idx(0);
    set_adc_ch_with_adc_ch_idx(adc_ch);
    adc_ch_on(adc_ch);
    wave_ch_on(0);

    delay_us(16);
    // turn_all_hc4067_off();
    adc_ch_off(adc_ch);
    wave_ch_off(0);

    HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);
    delay_us(200);
}

void main_hc4067_test(void)
{
    static uint8_t wave_ch = 0;
    static uint8_t adc_ch = 0;

    // wave_ch++;
    // if (wave_ch >= WAVE_CH_MAX)
    // {
    //     wave_ch = 0;
    //     adc_ch++;
    //     if (adc_ch >= ADC_CH_MAX)
    //     {
    //         adc_ch = 0; // 重置ADC通道
    //     }
    // }

    adc_ch++;
    if (adc_ch >= ADC_CH_MAX)
    {
        wave_ch++;
        if (wave_ch >= WAVE_CH_MAX)
        {
            wave_ch = 0; // 重置波形通道
        }
        adc_ch = 0; // 重置ADC通道
    }

    HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);

    start_dac_dma();
    set_wave_ch_with_wave_ch_idx(wave_ch);
    set_adc_ch_with_adc_ch_idx(adc_ch);

    adc_ch_on(adc_ch);
    wave_ch_on(wave_ch);

    delay_us(40);

    adc_ch_off(adc_ch);
    wave_ch_off(wave_ch);
    HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);

    delay_us(200);
}

void main_hc4067_test_(void)
{
    static uint8_t wave_ch = 0;
    static uint8_t adc_ch = 0;

    for (wave_ch = 0; wave_ch < WAVE_CH_MAX; wave_ch++)
    {
        // start_dac_dma();
        set_wave_ch_with_wave_ch_idx(wave_ch);
        turn_on_wave_ch(wave_ch);
        // turn_on_wave_ch(wave_ch);
        // delay_us(15);
        // turn_all_hc4067_off();
        for (adc_ch = 0; adc_ch < ADC_CH_MAX; adc_ch++)
        {
            HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);
            // 打开wave输出
            start_dac_dma();
            set_adc_ch_with_adc_ch_idx(adc_ch);
            turn_on_adc_ch(adc_ch);

            delay_us(15); // 等待一段时间

            // 关了
            HAL_GPIO_WritePin(HC4067_IN1_EN_GPIO_Port, HC4067_IN1_EN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(HC4067_IN2_EN_GPIO_Port, HC4067_IN2_EN_Pin, GPIO_PIN_SET);
            // turn_all_hc4067_off();

            HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);

            delay_us(80); // 等待一段时间
        }
    }
}

void main_wave_test(void)
{
    static uint8_t wave_ch = 0;
    static uint8_t adc_ch = 0;

    // for (adc_ch = 0; adc_ch < ADC_CH_MAX; adc_ch++)
    {
        set_adc_ch_with_adc_ch_idx(1);
        turn_on_adc_ch(1);

        for (wave_ch = 0; wave_ch < WAVE_CH_MAX; wave_ch++)
        {

            HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);

            // 打开wave输出
            start_dac_dma();

            set_wave_ch_with_wave_ch_idx(wave_ch);

            turn_on_wave_ch(wave_ch);

            delay_us(5); // 等待一段时间

            // turn_all_hc4067_off();

            HAL_GPIO_WritePin(HC4067_OUT1_EN_GPIO_Port, HC4067_OUT1_EN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(HC4067_OUT2_EN_GPIO_Port, HC4067_OUT2_EN_Pin, GPIO_PIN_SET);

            delay_us(100); // 等待一段时间

            HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);
            delay_us(10);
            // delay_us(80); // 等待一段时间
        }
    }
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

float low_pass_filter(float input, float prev_output, float alpha)
{
    return alpha * input + (1 - alpha) * prev_output;
}

uint32_t low_pass_filter_u32(uint32_t input, uint32_t prev_output, uint32_t a_num, uint32_t a_den)
{
    return (a_num * input + (a_den - a_num) * prev_output) / a_den;
}

void save_data_simple(uint16_t point_id)
{

    float adc_v = low_pass_filter((float)adc_mapped / ADC_SCALE, last_iir_adc_val[point_id], 0.1f);
    adc_final_result[point_id] = adc_v;
    last_iir_adc_val[point_id] = adc_v;
}

void save_data_directly(uint16_t point_id)
{
    uint16_t temp = (uint16_t)(adc_mapped / ADC_SCALE);
    if (temp > 0XFFFF)
    {
        temp = 0XFFFF;
    }
    adc_result_16bit_buf[point_id] = temp & 0xFFFF;
}

// 把数据从adc_final_result复制到tx发送缓存中
void cp_all_data_to_ram_u16(void)
{ 
    // memcpy((uint8_t *)adc_result_16bit, (uint8_t *)adc_result_16bit_buf, sizeof(uint16_t) * TOTAL_POINT_NUMBER);
    memset(adc_result_16bit, 0, sizeof(uint16_t) * TOTAL_POINT_NUMBER);
    uint16_t i = 0;
    for (i = 0; i < TOTAL_POINT_NUMBER; i++)
    {
        adc_result_16bit[i] = adc_result_16bit_buf[i];
    }
}

// 把数据从adc_final_result复制到tx发送缓存中
void cp_all_float_data_to_ram_u16(void)
{ 
    // memcpy((uint8_t *)adc_result_16bit, (uint8_t *)adc_result_16bit_buf, sizeof(uint16_t) * TOTAL_POINT_NUMBER);
    // memset(adc_result_16bit, 0, sizeof(uint16_t) * TOTAL_POINT_NUMBER);
    uint16_t i = 0;
    for (i = 0; i < TOTAL_POINT_NUMBER; i++)
    {
        // if (adc_final_result[i] > 65535.0f)
        // {
        //     adc_final_result[i] = 65535.0f;
        // }
        adc_result_16bit[i] = (uint16_t)adc_final_result[i];
    }
}

void save_data(void)
{
    static uint16_t point_idx = 0;

    float adc_v = low_pass_filter((float)adc_mapped / ADC_SCALE, last_iir_adc_val[point_idx], 0.1f);
    last_iir_adc_val[point_idx] = adc_v;
    adc_final_result[point_idx] = adc_v;

#if ENABLE_IIR

    // uint32_t adc_u32 = low_pass_filter_u32(adc_mapped, last_iir_adc_val_u32[point_idx], 1, 10);
    uint32_t adc_u32 = ema_u32(last_iir_adc_val_u32[point_idx], adc_mapped, 1, 10);

    // float adc_v = low_pass_filter((float)adc_mapped / ADC_SCALE, last_iir_adc_val[point_idx], 0.1f);
    // last_iir_adc_val[point_idx] = adc_v;
    adc_final_result[point_idx] = adc_v;

    // float adc_v = low_pass_filter((float)adc_mapped / ADC_SCALE, last_iir_adc_val[point_idx], IIR_ALPHA);
    // float adc_v = (float)adc_mapped / ADC_SCALE;

    // last_iir_adc_val[point_idx] = adc_v;
    last_iir_adc_val_u32[point_idx] = adc_u32;
    adc_final_result_u32[point_idx] = adc_u32;

    // 实时的
    // adc131_data_buf[point_idx] = adc_v;
    // adc_final_result[point_idx] = adc_v;

    // trace_baseline_u32(adc_u32, point_idx);

#else

#if EMA_DRIFT

    float adc_v = (float)adc_mapped / ADC_SCALE;
    // adc131_data_buf[point_idx] = adc_v;
    // 读旧值
    float b0 = global_ver.baseline[point_idx];
    float r0 = adc_v - b0; // 用旧基线算残差（用于输出与噪声）
    float absd = float_absdiff(adc_v, b0);

    if (!global_ver.initialized)
    {
        // 快速收敛阶段：建议仍输出去基线后的值，但可做限幅或标记 calibrating
        global_ver.baseline[point_idx] = ema_float(b0, adc_v, CALIB_BASELINE_A);
        global_ver.base_noise[point_idx] = ema_float(global_ver.base_noise[point_idx], absd, CALIB_NOISE_A);

        // 方案A：输出残差（推荐，别全置零）
        // adc131_data_buf[point_idx] = r0;
        adc_final_result[point_idx] = r0;
        // 方案B：需要静默可限幅：adc131_data_buf[point_idx] = clamp(r0, -CALIB_OUT_MAX, CALIB_OUT_MAX);
    }
    else
    {
        if (global_ver.state == ST_IDLE)
        {
            // 空闲：正常跟踪
            global_ver.baseline[point_idx] = ema_float(b0, adc_v, IDEL_BASELINE_A);
            global_ver.base_noise[point_idx] = ema_float(global_ver.base_noise[point_idx], absd, IDEL_NOISE_A);
        }

        // 输出用旧基线得到的残差（r0），保证噪声与输出一致
        // adc131_data_buf[point_idx] = r0;
        adc_final_result[point_idx] = r0;
    }
#else
    adc131_data_buf[point_idx] = (float)adc_mapped / ADC_SCALE;
#endif

#endif

    point_idx++;
    if (point_idx >= TOTAL_POINT_NUMBER)
    {
        // 指示周期开始
        // HAL_GPIO_TogglePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin);
        time_to_calculate = 1;
        point_idx = 0; // 重置索引
    }
}

void change_point_idx(void)
{
    point_idx++;
    if (point_idx > TOTAL_POINT_NUMBER)
    {
        // 指示周期开始
        // HAL_GPIO_TogglePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin);
        time_to_calculate = 1;
        point_idx = 0; // 重置索引
    }
}

// #include <sys/unistd.h> // write
// #include "uart5_log.h"

// int _write(int fd, const char *ptr, int len)
// {
//     (void)fd;
//     /* 非阻塞写队列；为保持 printf 语义，返回len（即使队列满可能丢尾部）。
//        若你想“真实返回已写入字节数”，把下面改成返回 U5Log_Write 的值即可。*/
//     size_t w = U5Log_Write((const uint8_t *)ptr, (size_t)len);
//     return (int)len; // 或 (int)w
// }

int fputc(int ch, FILE *f)
{
    uint8_t c = (uint8_t)ch;
    U5Log_Write(&c, 1);
    return ch;
}

void _sys_exit(int return_code)
{
    while (1)
        ; // 什么都不做
}
