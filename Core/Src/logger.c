// #include "user_comm.h"

// __attribute__((section("dma_buffer"), aligned(32)))
// uint8_t u5_tx_buf[128] = {0};

// void send_log(uint8_t *data, uint16_t len)
// {
//     static uint8_t counter = 0;
//     counter++;
//     memset(u5_tx_buf, 0, sizeof(u5_tx_buf));
//     u5_tx_buf[0] = counter;
//     // memcpy((uint8_t *)u5_tx_buf, data, len);
//     // HAL_UART_Transmit(&huart5, (uint8_t *)u5_tx_buf, len, 0xFFFF);
//     HAL_UART_Transmit_DMA(&huart5, (uint8_t *)u5_tx_buf, 10);
//     // HAL_UART_Transmit_DMA(&huart1, (uint8_t *)u5_tx_buf, len);
// }

// void print_hellow(void)
// {
//     char *str = "Hello from logger!\r\n";
//     send_log((uint8_t *)str, strlen(str));
// }

#include "user_comm.h"
#include <string.h>

/* ---------------- 配置区 ---------------- */
#define U5LOG_Q_SIZE 4096u    // 软件环形队列容量（建议4K或8K）
#define U5LOG_DMA_CHUNK 256u  // 每次DMA发送的最大分片
#define U5LOG_USE_CRITICAL 1  // 1: 使用临界区保护(建议)
#define U5LOG_AUTO_CRLF_DEF 1 // 默认把 '\n' 转 "\r\n"

/* H7: 把 DMA 缓冲放 D2 SRAM (0x3000_0000)，并32字节对齐 */
__attribute__((section("dma_buffer"), aligned(32))) static uint8_t u5_dma_buf[U5LOG_DMA_CHUNK];

static UART_HandleTypeDef *s_huart;
static volatile uint8_t s_dma_busy = 0;
static uint8_t s_auto_crlf = U5LOG_AUTO_CRLF_DEF;

/* 简单环形队列（放在普通RAM即可） */
static uint8_t s_q[U5LOG_Q_SIZE];
static volatile uint32_t s_q_head = 0; // 写入位置
static volatile uint32_t s_q_tail = 0; // 取出位置

/* --------- 可选临界区封装（裸机/FreeRTOS通用） --------- */
#if U5LOG_USE_CRITICAL
#include "core_cm7.h"
static uint32_t s_primask;
static inline void u5log_enter_critical(void)
{
    s_primask = __get_PRIMASK();
    __disable_irq();
}
static inline void u5log_exit_critical(void)
{
    if (!s_primask)
        __enable_irq();
}
#else
#define u5log_enter_critical()
#define u5log_exit_critical()
#endif

static inline uint32_t q_usable(void)
{
    uint32_t h = s_q_head, t = s_q_tail;
    return (t + U5LOG_Q_SIZE - h - 1) % U5LOG_Q_SIZE; // 预留1字节避免满=空
}

static inline uint32_t q_len(void)
{
    uint32_t h = s_q_head, t = s_q_tail;
    return (h + U5LOG_Q_SIZE - t) % U5LOG_Q_SIZE;
}

static void q_push_bytes(const uint8_t *src, uint32_t n)
{
    uint32_t h = s_q_head;
    uint32_t first = U5LOG_Q_SIZE - h;
    if (first > n)
        first = n;
    memcpy(&s_q[h], src, first);
    if (n > first)
        memcpy(s_q, src + first, n - first);
    s_q_head = (h + n) % U5LOG_Q_SIZE;
}

static uint32_t q_peek_linear(uint8_t *dst, uint32_t maxn)
{
    /* 把队列里连续的一段（不跨尾）拷到 dst，返回字节数 */
    uint32_t t = s_q_tail, h = s_q_head;
    if (t == h)
        return 0;
    uint32_t linear = (h > t) ? (h - t) : (U5LOG_Q_SIZE - t);
    if (linear > maxn)
        linear = maxn;
    memcpy(dst, &s_q[t], linear);
    return linear;
}

static void q_pop(uint32_t n)
{
    s_q_tail = (s_q_tail + n) % U5LOG_Q_SIZE;
}

/* 启动一次 DMA 发送（从队列取数据搬到u5_dma_buf并发DMA） */
static void u5log_kick_dma_unlocked(void)
{
    if (s_dma_busy)
        return;
    uint32_t take = q_peek_linear(u5_dma_buf, U5LOG_DMA_CHUNK);
    if (take == 0)
        return;

    // /* H7 D-Cache：DMA前清缓存，地址需32B对齐，长度也最好按32B上取整 */
    // SCB_CleanDCache_by_Addr((uint32_t *)(((uintptr_t)u5_dma_buf) & ~31u),
    //                         (take + 31u) & ~31u);

    if (HAL_OK == HAL_UART_Transmit_DMA(s_huart, u5_dma_buf, take))
    {
        s_dma_busy = 1;
        /* 真正从队列弹出要等 DMA 完成回调里确认成功后再做；
           但 HAL 会立刻copy count，数据已经在u5_dma_buf里，无需保留源拷贝。
           为了简化，我们在成功启动DMA后就弹出（常见做法）。
         */
        q_pop(take);
    }
}

void send_data_u5(void)
{
    u5log_enter_critical();
    s_dma_busy = 0;
    /* 队列里还有数据？继续发下一片 */
    u5log_kick_dma_unlocked();
    u5log_exit_critical();
}

/* 供外部调用：初始化 */
void U5Log_Init(UART_HandleTypeDef *huart5)
{
    s_huart = huart5;
    s_dma_busy = 0;
    s_q_head = s_q_tail = 0;
    s_auto_crlf = U5LOG_AUTO_CRLF_DEF;

    U5Log_SetAutoCRLF(1);
}

/* 可开关 '\n' -> "\r\n" */
void U5Log_SetAutoCRLF(uint8_t enable) { s_auto_crlf = enable ? 1 : 0; }

/* 写入接口：把数据压进环形队列，并尝试触发DMA */
size_t U5Log_Write(const uint8_t *data, size_t len)
{
    size_t written = 0;

    u5log_enter_critical();

    for (size_t i = 0; i < len; ++i)
    {
        uint8_t ch = data[i];

        if (s_auto_crlf && ch == '\n')
        {
            if (q_usable() == 0)
                break;
            uint8_t cr = '\r';
            q_push_bytes(&cr, 1);
            written++;
        }

        if (q_usable() == 0)
            break;
        q_push_bytes(&ch, 1);
        written++;
    }

    /* 若DMA空闲，则立即启动一次 */
    u5log_kick_dma_unlocked();

    u5log_exit_critical();
    return written;
}

// /* 可选：错误回调，发生错误时复位 busy 并尝试重启 */
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     if (huart != s_huart)
//         return;

//     u5log_enter_critical();
//     s_dma_busy = 0;
//     u5log_kick_dma_unlocked();
//     u5log_exit_critical();
// }

int myprintf(const char *format, ...)
{
    va_list arg;
    static char SendBuff[256] = {0};
    int rv;
    // while (!usart_dma_tx_over)
    //     ; // 等待前一次DMA发送完成

    va_start(arg, format);
    rv = vsnprintf((char *)SendBuff, sizeof(SendBuff) + 1, (char *)format, arg);
    va_end(arg);

    U5Log_Write((uint8_t *)SendBuff, rv);

    // HAL_UART_Transmit_DMA(&huart2, (uint8_t *)SendBuff, rv);
    // usart_dma_tx_over = 0; // 清0全局标志，发送完成后重新置1

    return rv;
}

// 正常打印示例：
// ads131 freq: 63748 /s
// ads131 freq: 63751 /s
// ads131 freq: 63762 /s
// ads131 freq: 63757 /s
// ads131 freq: 63762 /s
void ads131_frame_counter(void)
{
    uint32_t now_tck = HAL_GetTick();
    static uint32_t last_tick = 0;
    static uint32_t frame_counter = 0;
    frame_counter++;

    if (now_tck - last_tick > 1000)
    {

        printf("ads131 freq: %u /s\r\n", frame_counter);
        frame_counter = 0;
        last_tick = now_tck;
    }
}

