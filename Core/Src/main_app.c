#include "user_comm.h"

__IO uint32_t point5_val[PERIOD_FLAGS_MAX] = {0};

static uint16_t wave_ch = 0;
static uint16_t adc_ch = 0;
__IO uint8_t time_to_calculate = 0;
__IO uint8_t ready_to_calculate = 0;

__IO static uint32_t poroid_buf_5[PERIOD_FLAGS_MAX] = {0};
__IO static uint32_t poroid_buf_6[PERIOD_FLAGS_MAX] = {0};

void dac_20k_main_task_geer(void)
{
    static uint8_t peroid_counter = 0;
    if (ads131_data_ready == 0)
        return;

    get_ads_131_data();

    if (peroid_counter == 0)
    {
        start_dac_dma();
        set_wave_ch_with_wave_ch_idx(wave_ch);
        set_adc_ch_with_adc_ch_idx(adc_ch);
        wave_ch_on(wave_ch);
        adc_ch_on(adc_ch);
    }
    else if (peroid_counter == 3)
    {
        wave_ch_off(wave_ch);
        adc_ch_off(adc_ch);
    }
    else if (peroid_counter == 5)
    {
        save_data_directly(wave_ch + adc_ch * WAVE_CH_MAX);
        // save_data_simple(wave_ch + adc_ch * WAVE_CH_MAX);
    }

    // if (wave_ch == 5 && adc_ch == 0)
    // {
    //     poroid_buf_5[peroid_counter] = adc_mapped / 100;
    // }
    // else if (wave_ch == 6 && adc_ch == 0)
    // {
    //     poroid_buf_6[peroid_counter] = adc_mapped / 100;
    // }

    peroid_counter++;
    // 5个周期, 16*5即80us
    if (peroid_counter >= PERIOD_FLAGS_MAX)
    {

        change_hc4067_ch();
        if (time_to_calculate)
        {
            // cp_all_float_data_to_ram_u16();
            cp_all_data_to_ram_u16();
            time_to_calculate = 0;
        }
        // cp_all_float_data_to_ram_u16();
        // cp_all_data_to_ram_u16();
        peroid_counter = 0;
    }

    ads131_data_ready = 0;
}

void dac_20k_main_task_origin(void)
{
    static uint8_t peroid_counter = 0;
    if (ads131_data_ready == 0)
        return;

    get_ads_131_data();

    // 不用发送原生协议数据
    if (calculate_data())
    {
        return;
    }

    if (peroid_counter == 0)
    {
        start_dac_dma();
        set_wave_ch_with_wave_ch_idx(wave_ch);
        set_adc_ch_with_adc_ch_idx(adc_ch);
        wave_ch_on(wave_ch);
        adc_ch_on(adc_ch);
    }
    else if (peroid_counter == 3)
    {
        wave_ch_off(wave_ch);
        adc_ch_off(adc_ch);
    }
    else if (peroid_counter == 5)
    {
        save_data_simple(wave_ch + adc_ch * WAVE_CH_MAX);
    }

    peroid_counter++;
    // 5个周期, 16*5即80us
    if (peroid_counter >= PERIOD_FLAGS_MAX)
    {

        change_hc4067_ch();
        // cp_all_data_to_ram();
        peroid_counter = 0;
    }

    ads131_data_ready = 0;
}

uint8_t calculate_data(void)
{
    if (time_to_calculate == 0)
        return 0;

    // temp_drift_process();

    // 初始化检查
    // check_initialized();

    // check state
    // check_state();
    // HAL_GPIO_TogglePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin);
    send_ads131_val_to_master();
    // send_ads131_val_to_udp();
    // printf("P0: %u\r\n", adc_final_result_u32[0]);

    // printf("baseline: %u noise: %u \r\n", global_ver.baseline_u32[0], global_ver.basenoise_u32[0]);
    time_to_calculate = 0;
    return 1;
}

void change_hc4067_ch(void)
{

    wave_ch++;
    if (wave_ch >= WAVE_CH_MAX)
    {
        adc_ch++;
        if (adc_ch >= ADC_CH_MAX)
        {
            time_to_calculate = 1;

            adc_ch = 0; // 重置ADC通道
        }
        wave_ch = 0; // 重置波形通道
    }

    // wave_ch--;
    // if (wave_ch == 0)
    // {
    //     wave_ch = WAVE_CH_MAX;
    //     adc_ch++;
    //     if (adc_ch >= ADC_CH_MAX)
    //     {
    //         adc_ch = 0; // 重置ADC通道
    //     }
    // }

    // wave_ch++;
    // if (wave_ch >= WAVE_CH_MAX)
    // {
    //     adc_ch++;
    //     if (adc_ch >= ADC_CH_MAX)
    //     {
    //         adc_ch = 0; // 重置ADC通道
    //     }
    //     wave_ch = 0; // 重置波形通道
    // }

    // adc_ch++;
    // if (adc_ch >= ADC_CH_MAX)
    // {
    //     adc_ch = 0;
    //     wave_ch++;
    //     if (wave_ch >= WAVE_CH_MAX)
    //     {
    //         wave_ch = 0; // 重置波形通道
    //     }
    // }

    // return wave_ch * ADC_CH_MAX + adc_ch; // 返回当前的通道组合
}

void geer_main_task(void)
{
    geer_comm_handler();

    dac_20k_main_task_geer();

    data_upload_handle();
}

#if 0

// 16us*1024 = 16.384ms*3 = 49.152ms
void main_task(void)
{

    static uint8_t peroid_counter = 0;
    // __IO uint16_t point_id = 0;

    if (if_getting_data() == 0)
    {
        return;
    }

    if (ads131_data_ready == 0)
    {
        return;
    }

    // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);
    // 先采集数据
    get_ads_131_data();

    if (calculate_data())
    {
        return;
    }

    // if (wave_ch == 5 && adc_ch == 0)
    // {
    //     poroid_buf_5[peroid_counter] = adc_mapped / 100;
    // }
    // else if (wave_ch == 6 && adc_ch == 0)
    // {
    //     poroid_buf_6[peroid_counter] = adc_mapped / 100;
    // }

    if (peroid_counter == 0)
    {

        // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_SET);
        // peroid 0 切换输入输出通道
        start_dac_dma();
        set_wave_ch_with_wave_ch_idx(wave_ch);
        set_adc_ch_with_adc_ch_idx(adc_ch);
        wave_ch_on(wave_ch);
        adc_ch_on(adc_ch);
    }
    else if (peroid_counter == 1)
    {
        // 关门
        // turn_all_hc4067_off();
        // start_dac_dma();
        wave_ch_off(wave_ch);
        adc_ch_off(adc_ch);
    }
    else if (peroid_counter == 5)
    {
        // point_id = wave_ch + adc_ch * WAVE_CH_MAX;
        save_data_simple(wave_ch + adc_ch * WAVE_CH_MAX);
        // change_hc4067_ch();
    }

    // if (wave_ch == 5 && adc_ch == 0)
    // {
    //     point5_val[peroid_counter] = adc_mapped;
    // }

    // 先获取数据, 无论哪个周期, 都要获取数据
    // get_ads_131_data();

    // step 3 获取数据
    // 7us是极限了, 再多会有问题.
    // delay_us(7); // delay以获取最佳线性度
    // if (peroid_counter == 0)
    // {
    //     // delay_us(7);
    //     // 关门
    //     // turn_all_hc4067_off();
    // }

    peroid_counter++;
    // 周期为0-5
    if (peroid_counter >= PERIOD_FLAGS_MAX)
    {
        change_hc4067_ch();

        cp_all_data_to_ram();
        // wave_ch++;
        // if (wave_ch >= WAVE_CH_MAX)
        // {
        //     wave_ch = 0;
        // }
        // // HAL_GPIO_WritePin(TEST_PORT_2_GPIO_Port, TEST_PORT_2_Pin, GPIO_PIN_RESET);
        // if (ready_to_calculate)
        // {
        //     ready_to_calculate = 0;
        //     time_to_calculate = 1;
        // }
        peroid_counter = 0; // 重置周期计数器
    }

    ads131_data_ready = 0;
}

void main_app(void)
{

    static uint8_t peroid_counter = 0;

    if (ads131_data_ready == 0)
        return;

    // ads131_frame_counter();

    // 先采集数据
    get_ads_131_data();

    if (calculate_data())
    {
        return;
    }
    if (peroid_counter == 0)
    {
        change_hc4067_ch();

        start_dac_dma();
    }
    else if (peroid_counter == 1)
    {
        turn_all_hc4067_off();
    }
    else if (peroid_counter == 5)
    {
        save_data();
    }

    peroid_counter++;
    if (peroid_counter >= PERIOD_FLAGS_MAX)
    {
        // save_data();
        change_point_idx();
        peroid_counter = 0; // 重置周期计数器
    }

    ads131_data_ready = 0;
}
#endif

__IO static uint32_t poroid_buf[PERIOD_FLAGS_MAX] = {0};

// 事实证明, 0-5, 第5个周期的值最大.
void main_app_ads_peroid_test(void)
{
    static uint8_t flag = 0;
    static uint8_t wave_ch = 5;
    static uint8_t adc_ch = 0;

    static uint8_t peroid_counter = 0;

    if (ads131_data_ready == 0)
        return;

    // ads131_frame_counter();

    // 先采集数据
    get_ads_131_data();

    if (wave_ch == 5)
    {
        poroid_buf_5[peroid_counter] = adc_mapped / 100;
    }
    else if (wave_ch == 6)
    {
        poroid_buf_6[peroid_counter] = adc_mapped / 100;
    }

    poroid_buf[peroid_counter] = adc_mapped / 100;

    if (peroid_counter == 0)
    {
        start_dac_dma();
        set_wave_ch_with_wave_ch_idx(wave_ch);
        set_adc_ch_with_adc_ch_idx(adc_ch);
        wave_ch_on(wave_ch);
        adc_ch_on(adc_ch);
    }
    else if (peroid_counter == 1)
    {
        // start_dac_dma();
        wave_ch_off(wave_ch);
        adc_ch_off(adc_ch);
    }

    peroid_counter++;
    if (peroid_counter >= PERIOD_FLAGS_MAX)
    {
        peroid_counter = 0;

        if (flag == 0)
        {
            flag = 1;
            wave_ch = 5;
        }
        else if (flag == 1)
        {
            flag = 2;
            wave_ch = 6;
        }
        else if (flag == 2)
        {
            flag = 0;
            wave_ch = 7;
        }
    }

    ads131_data_ready = 0;
}
