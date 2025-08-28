#include "user_comm.h"

// ---------- 参数区（按需调优） ----------
#define EPS 1e-12f
#define C_SIGMA_FROM_MEANABS 1.2533141f // σ ≈ 1.2533 * E|r|
#define K_SIG 3.0f                      // 自适应阈：|r| 与 σ 的相对尺度
#define K_ERR 3.0f
#define ALPHA_MIN 0.02f                 // 静态最小步进（更小=更稳）
#define ALPHA_MAX 0.50f                 // 大信号时最大步进（更大=更跟手）
#define NOISE_ALPHA 0.05f               // 噪声 EMA 的 α（越小越稳）
#define NOISE_WINSOR_C 4.0f             // winsorize 上限：min(|r|, C*σ)
#define BASELINE_ALPHA_TRACK 0.01f      // 基线慢速跟踪的 α（很小）
#define T_TRACK 1.5f                    // 仅当 |r0| < T_TRACK*σ 才更新基线
#define DEAD_BAND_K 0.5f                // 死区倍数（相对 σ）

// 事件判定（可选）
#define PRESS_K_ON 3.0f        // 上阈：K_on*σ
#define PRESS_K_OFF 2.0f       // 下阈：K_off*σ（<K_on 形成滞回）
#define DEBOUNCE_ON_SAMPLES 3  // 连续 n 个样本超阈才算按下
#define DEBOUNCE_OFF_SAMPLES 3 // 连续 n 个样本低于下阈才算释放


// ---------- 工具函数 ----------
static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
static inline float ema_step(float prev, float x, float a)
{
    return prev + a * (x - prev);
}
static inline float median3f(float a, float b, float c)
{
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
    return b; // 中值
}

// ---------- 状态（每通道一份） ----------
typedef struct
{
    float baseline;
    float mean_abs_r; // ≈ E|r|，用来推 σ
    float r_prev1, r_prev2;
    float y_prev;    // 平滑输出上一帧
    int initialized; // 冷启动标志

    // 事件判定（可选）
    int pressed;
    int on_cnt, off_cnt;
} ch_state_t;

ch_state_t ch_states[TOTAL_SENSOR_NUMBER] = {0};

// ---------- 单点处理：输入 adc_mapped，输出 y（已平滑的去基线值） ----------
float process_point_step(ch_state_t *st, uint16_t adc_mapped)
{
    // 0) 原始值 → 浮点
    float x = (float)adc_mapped;

    // 冷启动：第一帧直接对齐
    if (!st->initialized)
    {
        st->baseline = x;
        st->mean_abs_r = 0.0f; // 刚开始先当作无噪声，后续快速收敛
        st->r_prev2 = st->r_prev1 = 0.0f;
        st->y_prev = 0.0f;
        st->pressed = 0;
        st->on_cnt = st->off_cnt = 0;
        st->initialized = 1;
        return 0.0f; // 冷启动首帧输出 0
    }

    // 1) 残差（用旧基线）
    float b0 = st->baseline;
    float r0 = x - b0;

    // 2) 中值3 抗毛刺
    float r_med = median3f(st->r_prev2, st->r_prev1, r0);

    // 3) 噪声估计（winsorize 再 EMA）
    float sigma = C_SIGMA_FROM_MEANABS * fmaxf(st->mean_abs_r, EPS);
    float absr = fabsf(r0);
    float absr_w = fminf(absr, NOISE_WINSOR_C * sigma);
    st->mean_abs_r = ema_step(st->mean_abs_r, absr_w, NOISE_ALPHA);
    sigma = C_SIGMA_FROM_MEANABS * fmaxf(st->mean_abs_r, EPS); // 更新后的 σ

    // 4) 自适应 α
    // float ratio = (K_SIG * sigma > EPS) ? fabsf(r_med) / (K_SIG * sigma) : 1.0f;
    // ratio = clampf(ratio, 0.0f, 1.0f);
    // float alpha = ALPHA_MIN + (ALPHA_MAX - ALPHA_MIN) * ratio;

    float err = fabsf(r_med - st->y_prev);
    float ratio_err = (K_ERR * sigma > EPS) ? err / (K_ERR * sigma) : 1.0f;

    // 叠加策略：取 max，谁大用谁
    float ratio_amp = (K_SIG * sigma > EPS) ? fabsf(r_med) / (K_SIG * sigma) : 1.0f;
    float ratio = fmaxf(ratio_amp, ratio_err);
    ratio = clampf(ratio, 0.0f, 1.0f);

    float alpha = ALPHA_MIN + (ALPHA_MAX - ALPHA_MIN) * ratio;

    // 5) 一阶低通
    float y = ema_step(st->y_prev, r_med, alpha);

    // 6) 死区
    float dead = DEAD_BAND_K * sigma;
    if (fabsf(y) < dead)
        y = 0.0f;

    // 7) 基线“条件跟踪”（只在小残差时，慢慢跟）
    if (fabsf(r0) < T_TRACK * sigma)
    {
        st->baseline = ema_step(b0, x, BASELINE_ALPHA_TRACK);
    }
    // （否则冻结或极慢跟踪，避免被大信号拖走）

    // 8) 更新历史 & 输出
    st->r_prev2 = st->r_prev1;
    st->r_prev1 = r0;
    st->y_prev = y;

    return y; // 去基线且平滑后的值
}

void save_adc_data_(void)
{

    adc_final_result[point_idx] = process_point_step(&ch_states[point_idx], adc_mapped / ADC_SCALE);
}

// ---------- 事件判定（可选） ----------
int process_point_event(ch_state_t *st, float y, float sigma)
{
    float th_on = PRESS_K_ON * sigma;
    float th_off = PRESS_K_OFF * sigma;

    if (!st->pressed)
    {
        if (fabsf(y) > th_on)
        {
            if (++st->on_cnt >= DEBOUNCE_ON_SAMPLES)
            {
                st->pressed = 1;
                st->off_cnt = 0;
            }
        }
        else
        {
            st->on_cnt = 0;
        }
    }
    else
    {
        if (fabsf(y) < th_off)
        {
            if (++st->off_cnt >= DEBOUNCE_OFF_SAMPLES)
            {
                st->pressed = 0;
                st->on_cnt = 0;
            }
        }
        else
        {
            st->off_cnt = 0;
        }
    }
    return st->pressed;
}
