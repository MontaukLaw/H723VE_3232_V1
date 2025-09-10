#include "user_comm.h"

#define TRACK_WINDOW_CODE 8.0f // 小偏差窗口（单位：ADC码）。应大于噪声RMS的 3~5 倍
#define BASELINE_ALPHA 0.003f  // 基线跟随速度：0.001~0.01
#define OUT_LP_BETA 0.3f       // 输出轻微低通（0~1），不想低通就设为1
#define INIT_SAMPLES 200       // 初始化收集样本数

#define LP_ALPHA_MIN 0.02f // 平稳时最小更新系数（更小=更平滑）
#define LP_ALPHA_MAX 0.5f  // 有效信号时最大更新系数（更大=更跟手）
#define NOISE_MULT_K 3.0f  // 噪声倍数，|r0|>K*σ 认为是“真信号”
#define DEAD_BAND_K 0.5f   // 死区阈值的倍数系数（相对 σ）

float r_prev1[TOTAL_SENSOR_NUMBER]; // 残差 r 的上一帧
float r_prev2[TOTAL_SENSOR_NUMBER]; // 残差 r 的上上帧
float y_filt[TOTAL_SENSOR_NUMBER];  // 平滑后的输出（上一帧）

extern g_ver global_ver;

static uint8_t inited = 0;
static uint16_t init_cnt = 0;
static float baseline[TOTAL_SENSOR_NUMBER];
static float out_lp[TOTAL_SENSOR_NUMBER]; // 输出低通状态，可选

float adc_final_result[TOTAL_SENSOR_NUMBER];

#define NOISE_K 1.8f               // 1.5~3.0 之间调；越大越“干净”但越钝
#define NOISE_TMIN 0.0f            // 最小门限，防止极低噪声时抖动，可设为几 LSB
#define ACTIVE_NOISE_A 0.02f       // 非 IDLE 时也轻微更新噪声，避免“冻结”
#define ACTIVE_BASELINE_LEAK 0.01f // 非 IDLE 时给基线一点点“泄放”，回落更跟手

// tools
float ema_float(float y, float x, float a) { return y + a * (x - y); }
// uint32_t ema_u32(uint32_t y, uint32_t x, uint32_t a, uint32_t b)
// {
//     return y + (a * (x - y)) / b;
// }

uint32_t ema_u32(uint32_t prev, uint32_t x, uint32_t a_num, uint32_t a_den)
{
    // y = (a)*prev + (1-a)*x, 其中 a = a_num/a_den
    // 用 64 位避免溢出，并做 +a_den/2 实现四舍五入，降低系统性偏差
    uint64_t num = (uint64_t)prev * a_num + (uint64_t)x * (a_den - a_num);
    return (uint32_t)((num + (a_den >> 1)) / a_den);
}

float float_absdiff(float a, float b) { return (a > b) ? (a - b) : (b - a); }

uint32_t u32_absdiff(uint32_t a, uint32_t b) { return (a > b) ? (a - b) : (b - a); }

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float lpf_step(float y_prev, float x, float a) { return y_prev + a * (x - y_prev); }
static inline float median3f(float a, float b, float c)
{
    // 返回 a,b,c 的中间值
    if (a > b)
    {
        float t = a;
        a = b;
        b = t;
    }
    if (b > c)
    {
        float t = b;
        b = c;
        c = t;
    }
    if (a > b)
    {
        float t = a;
        a = b;
        b = t;
    }
    return b; // 此时 b 为中值
}

void check_state(void)
{
    static uint8_t touch_cnt = 0;
    static uint8_t idle_cnt = 0;
    uint8_t touching = 0;

    if (global_ver.initialized == false)
        return;

    // 检测是否有触摸
    for (uint16_t i = 0; i < TOTAL_SENSOR_NUMBER; i++)
    {
        if (adc_final_result[i] > (TOUCHING_THRESHOLD_MULTIPLIER * global_ver.base_noise[i]))
        {
            touching = 1;
            break;
        }
    }

    // 状态机：消抖
    if (touching)
    {
        touch_cnt++;
        idle_cnt = 0;
        if (touch_cnt >= 3)
        {
            global_ver.state = ST_TOUCHING;
            touch_cnt = 3; // 防止溢出
        }
    }
    else
    {
        idle_cnt++;
        touch_cnt = 0;
        if (idle_cnt >= 3)
        {
            global_ver.state = ST_IDLE;
            idle_cnt = 3; // 防止溢出
        }
    }
}

void check_initialized(void)
{
    static uint8_t init_time = 0;
    if (!global_ver.initialized)
    {
        init_time++;
        if (init_time > INIT_FRAMES)
        {
            global_ver.initialized = true;
            printf("System initialized >>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
        }
    }
}

#if 0
void temp_drift_process(void)
{
    // 初始化期：在线均值作为初始基线
    if (!inited)
    {
        for (uint16_t i = 0; i < TOTAL_SENSOR_NUMBER; i++)
        {
            float x = (float)adc131_data_buf[i];
            baseline[i] += (x - baseline[i]) / (float)(init_cnt + 1);
            out_lp[i] = 0.0f;
            adc_final_result[i] = 0.0f;
        }
        if (++init_cnt >= INIT_SAMPLES)
            inited = 1;
        return;
    }

    // 正常期
    for (uint16_t i = 0; i < TOTAL_SENSOR_NUMBER; i++)
    {
        float x = (float)adc131_data_buf[i];
        float err = x - baseline[i]; // 带符号差值，别取绝对值

        // 条件跟随：小偏差->温漂，缓慢跟随；大偏差->真信号，冻结
        if (err > -TRACK_WINDOW_CODE && err < TRACK_WINDOW_CODE)
        {
            baseline[i] += BASELINE_ALPHA * err;
        }

        // 轻微低通（可选）
        float y = out_lp[i] + OUT_LP_BETA * (err - out_lp[i]);
        out_lp[i] = y;

        adc_final_result[i] = y; // 趋近 0（均值≈0），抖动≈噪声
    }
}



float adc_zero_val_buf[TOTAL_SENSOR_NUMBER][ZERO_INIT_TIMES] = {0.0f}; // 零点值缓存
float adc_zero_val[TOTAL_SENSOR_NUMBER] = {0.0f};                      // 零点值


void temp_drift_process(void)
{
    uint16_t i;
    float diff = 0.0f;
    static uint8_t if_init = 0;
    static uint16_t running_init_zero = 0;
    float sum = 0;

    // 未初始化阶段, 捕捉零值, 零值的缓冲长度为 ZERO_INIT_TIMES
    if (if_init == 0)
    {
        for (i = 0; i < TOTAL_SENSOR_NUMBER; i++)
        {
            adc_zero_val_buf[i][running_init_zero] = adc131_data_buf[i];
        }
    }
    else
    {
        // 初始化完成后,
        for (i = 0; i < TOTAL_SENSOR_NUMBER; i++)
        {

            diff = adc131_data_buf[i] - adc_zero_val[i];

            if (diff < 0)
            {
                diff = -diff; // 取绝对值
            }

            // 判断变化范围, 如果不大, 进入零值跟踪
            if (diff < AUTO_AGC_THRESHOLD)
            {
                adc_zero_val_buf[i][running_init_zero] = adc131_data_buf[i];
            }
        }
    }

    // 对零值求平均, 得到零值的平均值adc_zero_val
    for (i = 0; i < TOTAL_SENSOR_NUMBER; i++)
    {
        for (uint16_t j = 0; j < ZERO_INIT_TIMES; j++)
        {
            sum += adc_zero_val_buf[i][j];
        }
        adc_zero_val[i] = (float)(sum / ZERO_INIT_TIMES);
    }

    // 初始化完成后, 计算与零值的差异得最终结果.
    if (if_init)
    {
        for (i = 0; i < TOTAL_SENSOR_NUMBER; i++)
        {
            diff = adc131_data_buf[i] - adc_zero_val[i];

            if (diff < 0)
            {
                diff = -diff; // 取绝对值
            }

            adc_final_result[i] = diff; // 计算最终结果
        }
    }

    running_init_zero++;
    if (running_init_zero >= ZERO_INIT_TIMES)
    {
        if_init = 1;
        running_init_zero = 0;
    }
}

#endif

static inline float soft_threshold(float x, float t)
{
    float ax = fabsf(x);
    if (ax <= t)
        return 0.0f;
    return copysignf(ax - t, x);
}

void save_adc_data(void)
{

    float adc_v = (float)adc_mapped / ADC_SCALE;
    adc_final_result[point_idx] = adc_v;
    return;

    // adc131_data_buf[point_idx] = adc_v;
    // 读旧值
    float b0 = global_ver.baseline[point_idx];
    float r0 = adc_v - b0; // 用旧基线算残差（用于输出与噪声）
    float absd = float_absdiff(adc_v, b0);
    float n0 = global_ver.base_noise[point_idx];

    if (!global_ver.initialized)
    {
        // 快速收敛阶段：建议仍输出去基线后的值，但可做限幅或标记 calibrating
        global_ver.baseline[point_idx] = ema_float(b0, adc_v, CALIB_BASELINE_A);
        global_ver.base_noise[point_idx] = ema_float(global_ver.base_noise[point_idx], absd, CALIB_NOISE_A);

        // 方案A：输出残差（推荐，别全置零）
        // adc131_data_buf[point_idx] = r0;
        // adc_final_result[point_idx] = r0;
        float thr = fmaxf(NOISE_TMIN, NOISE_K * n0);
        adc_final_result[point_idx] = soft_threshold(r0, thr);
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
        float thr = fmaxf(NOISE_TMIN, NOISE_K * n0);
        adc_final_result[point_idx] = soft_threshold(r0, thr);

        // adc_final_result[point_idx] = r0;
    }
}

void trace_baseline(float adc_v, uint16_t point_idx)
{
    // 读旧值
    float b0 = global_ver.baseline[point_idx];
    float r0 = adc_v - b0; // 用旧基线算残差（用于输出与噪声）
    float absd = float_absdiff(adc_v, b0);
    float n0 = global_ver.base_noise[point_idx];
    if (!global_ver.initialized)
    {
        global_ver.baseline[point_idx] = ema_float(b0, adc_v, 0.8f);
        global_ver.base_noise[point_idx] = ema_float(n0, absd, 0.8f);
    }
    else
    {
        global_ver.baseline[point_idx] = ema_float(b0, adc_v, 0.01f);
        global_ver.base_noise[point_idx] = ema_float(n0, absd, 0.01f);
    }
}

void trace_baseline_u32_(uint32_t adc_v, uint16_t point_idx)
{
    // 读旧值
    uint32_t b0 = global_ver.baseline_u32[point_idx];
    uint32_t r0 = adc_v - b0; // 用旧基线算残差（用于输出与噪声）
    uint32_t absd = u32_absdiff(adc_v, b0);
    uint32_t n0 = global_ver.basenoise_u32[point_idx];

    if (!global_ver.initialized)
    {
        global_ver.baseline_u32[point_idx] = ema_u32(b0, adc_v, 3, 4);
        global_ver.basenoise_u32[point_idx] = ema_u32(n0, absd, 3, 4);
    }
    else
    {
        global_ver.baseline_u32[point_idx] = ema_u32(b0, adc_v, 1, 100);
        global_ver.basenoise_u32[point_idx] = ema_u32(n0, absd, 1, 100);
    }
}

void trace_baseline_u32(uint32_t adc_v, uint16_t point_idx)
{
    uint32_t b0 = global_ver.baseline_u32[point_idx];
    uint32_t n0 = global_ver.basenoise_u32[point_idx];

    // 1) 自适应基线：小残差快跟、大残差慢跟
    uint32_t resid0 = (adc_v > b0) ? (adc_v - b0) : (b0 - adc_v);
    uint32_t aB_num, aB_den;
    if (!global_ver.initialized)
    {
        aB_num = 1;
        aB_den = 8;
    } // init: 快
    else if (resid0 < (n0 << 2))
    {
        aB_num = 1;
        aB_den = 64;
    } // 静稳区：中等
    else
    {
        aB_num = 1;
        aB_den = 512;
    } // 有信号：很慢

    uint32_t b1 = ema_u32(b0, adc_v, aB_num, aB_den);
    global_ver.baseline_u32[point_idx] = b1;

    // 2) 用“新基线”算残差
    uint32_t absd = (adc_v > b1) ? (adc_v - b1) : (b1 - adc_v);

    // 3) 限幅/门控：避免把信号当噪声学进去（winsorize）
    //    允许每次最多比当前噪声上升 50%（可调），也设一个噪声地板
    const uint32_t NOISE_FLOOR = 1; // 视 ADC 量化/环境而定
    uint32_t rise_cap = n0 + (n0 >> 1) + NOISE_FLOOR;
    uint32_t absd_clamped = (absd < rise_cap) ? absd : rise_cap;

    // 4) 噪声“慢涨快降”
    uint32_t aN_up_num = 1, aN_up_den = 64; // 上升很慢
    uint32_t aN_dn_num = 1, aN_dn_den = 16; // 下降较快

    uint32_t n1;
    if (absd_clamped > n0)
        n1 = ema_u32(n0, absd_clamped, aN_up_num, aN_up_den);
    else
        n1 = ema_u32(n0, absd_clamped, aN_dn_num, aN_dn_den);

    if (n1 < NOISE_FLOOR)
        n1 = NOISE_FLOOR;

    global_ver.basenoise_u32[point_idx] = n1;

}
