#include "user_comm.h"

__attribute__((section("dma_buffer"), aligned(32))) static response_packet_t resp_packet_s;
static __IO uint16_t frame_id = 0;
static __IO uint8_t got_rx = 0;
static __IO uint8_t if_get_data = 0;

float adc_final_result[TOTAL_SENSOR_NUMBER];
const uint8_t HEAD_DATA[] = {0xff, 0xff, 0x06, 0x09};
const uint8_t DEVICE_INFO_DATA[] = {0x53, 0x59, 0x43, 0x4D, 0x37, 0x32, 0x33, 0x2D, 0x31, 0x30, 0x32, 0x34, 0x00};

__attribute__((section("dma_buffer"), aligned(32)))
uint8_t uart1_rx_buf[STANDARD_PROTOCAL_LEN];

static void trans_uint16_to_uint8(uint8_t *data, uint16_t len)
{
    // 这个还挺重要的. 一定要清零
    memset(data, 0, len);

    uint16_t adc_result_idx = 0;

    float flo_part = 0;
    uint32_t int_part = 0;
    uint32_t flo_to_int = 0;

    for (adc_result_idx = 0; adc_result_idx < TOTAL_SENSOR_NUMBER; adc_result_idx++)
    {
        int_part = (uint32_t)(adc131_data_buf[adc_result_idx]);
        data[adc_result_idx * PER_SENSOR_DATA_LEN + 0] = (int_part & 0xFF0000) >> 16;
        data[adc_result_idx * PER_SENSOR_DATA_LEN + 1] = (int_part & 0xFF00) >> 8;
        data[adc_result_idx * PER_SENSOR_DATA_LEN + 2] = int_part & 0x00FF;

        flo_part = adc131_data_buf[adc_result_idx] - int_part;
        flo_to_int = (uint32_t)(flo_part * 1000000); // 放大成整数处理

        // 小数部分
        data[adc_result_idx * PER_SENSOR_DATA_LEN + 3] = (flo_to_int & 0xFF0000) >> 16;
        data[adc_result_idx * PER_SENSOR_DATA_LEN + 4] = (flo_to_int & 0x00FF00) >> 8;
        data[adc_result_idx * PER_SENSOR_DATA_LEN + 5] = flo_to_int & 0x00FF;
    }
}

static void send_fb_to_master(void)
{

    uint16_t len = resp_packet_s.frame_data.data_seg_len + sizeof(resp_packet_s.frame_data.cmd_id) + sizeof(resp_packet_s.frame_data.target_id) + sizeof(resp_packet_s.frame_data.device_id) + sizeof(resp_packet_s.frame_data.status_id);

    // 大小端换一下
    resp_packet_s.frame_len = len >> 8 | len << 8;

    resp_packet_s.whole_packet_len = len + sizeof(resp_packet_s.frame_id) + sizeof(resp_packet_s.frame_len) + sizeof(resp_packet_s.head);

    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&resp_packet_s, resp_packet_s.whole_packet_len);

    uart1_busy = 1; // 设置发送标志
}

static uint32_t get_device_id(void)
{
    return DEVICE_ID;
}

void init_response_packet(response_packet_t *packet)
{ 
    memset(packet, 0, sizeof(response_packet_t));
    // always same head
    memcpy(packet->head, HEAD_DATA, 4);

    packet->frame_id = frame_id++;

    packet->frame_data.device_id = get_device_id();
    packet->frame_data.target_id = TARGET_ID;
    // 这里要返回传感器数量
    packet->frame_data.status_id = SENSOR_NUMBER_HEX;

    packet->frame_data.data_seg_len = TOTAL_SENSOR_NUMBER * PER_SENSOR_DATA_LEN;
}

void make_data_ready(void)
{
    init_response_packet(&resp_packet_s);

    resp_packet_s.frame_data.cmd_id = START_TRANS_RESP;
    trans_uint16_to_uint8(resp_packet_s.frame_data.data_seg, resp_packet_s.frame_data.data_seg_len);
    send_fb_to_master();
}

void send_device_info_to_master(void)
{

    init_response_packet(&resp_packet_s);

    resp_packet_s.frame_data.cmd_id = QUERY_DEVICE_ID_RESP;

    resp_packet_s.frame_data.status_id = STANDARD_CMD_STATUS_SUCESS;

    resp_packet_s.frame_data.data_seg_len = sizeof(DEVICE_INFO_DATA);

    memcpy(resp_packet_s.frame_data.data_seg, DEVICE_INFO_DATA, sizeof(DEVICE_INFO_DATA));

    send_fb_to_master();
}

void start_get_data(void)
{
    if (if_get_data == 0)
    {
        if_get_data = 1;
    }
}

void send_data_init_fb_to_master(void)
{

    init_response_packet(&resp_packet_s);

    resp_packet_s.frame_data.cmd_id = INIT_DEVICE_RESP;

    trans_uint16_to_uint8(resp_packet_s.frame_data.data_seg, resp_packet_s.frame_data.data_seg_len);

    send_fb_to_master();
}

void stop_get_data(void)
{
    if_get_data = 0;
}

void send_data_type_to_master(void)
{

    init_response_packet(&resp_packet_s);

    resp_packet_s.frame_data.cmd_id = SYNC_DATA_TYPE_RESP;

    // L个字节
    resp_packet_s.frame_data.status_id = 0x0300;

    resp_packet_s.frame_data.data_seg_len = 0;

    send_fb_to_master();
}

void cmd_handler(uint8_t *cmd_buffer)
{

    // 第一步, 检查包头是否为0xFF 0xFF 0x06 0x09
    if (cmd_buffer[0] != 0xFF || cmd_buffer[1] != 0xFF || cmd_buffer[2] != 0x06 || cmd_buffer[3] != 0x09)
    {
        return;
    }

    uint16_t cmd_id = ((uint16_t)cmd_buffer[16]) << 8 | ((uint16_t)cmd_buffer[17]);

    switch (cmd_id)
    {
    case QUERY_DEVICE_ID:
        send_device_info_to_master();
        break;

    case INIT_DEVICE:
        send_data_init_fb_to_master();
        break;

    case START_TRANS:
        start_get_data();
        break;

    case STOP_TRANS:
        stop_get_data();
        break;

    case SYNC_DATA_TYPE:
        send_data_type_to_master();
        break;
    }
}

void recv_handler(uint16_t len)
{
    static uint16_t frame_len = 0;
    frame_len = ((uint16_t)uart1_rx_buf[6]) << 8 | (uint16_t)uart1_rx_buf[7];

    if (frame_len + 8 == len)
    {
        cmd_handler(uart1_rx_buf);
    }

    memset(uart1_rx_buf, 0, UART_RX_BUF_LEN);
}

void comm_handler(void)
{
    // 收到RTX中断
    // 收到RTX中断
    // if (got_rx && uart_sending == 0)
    if (got_rx)
    {
        recv_handler(got_rx);
        got_rx = 0;
    }
}

void start_uart_rx(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, UART_RX_BUF_LEN);
}

void uart1_it_handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        // 清除 IDLE 标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        // 此时说明可能一帧数据接收完了
        // 获取当前 DMA 计数器剩余量, 计算本次接收字节数
        uint16_t remain = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        if (remain)
        {
            uint16_t data_len = UART_RX_BUF_LEN - remain;
            got_rx = data_len;
        }

        // 处理完毕后，可以选择清零或记录这段数据
        // 然后继续保持 DMA 接收，让它随时准备接收新的数据
        HAL_UART_AbortReceive(&huart1);
        // HAL_UART_DMAStop(&huart1);
        // 重新开启接收
        HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, UART_RX_BUF_LEN);
    }
}

void send_ads131_val_to_master(void)
{
    init_response_packet(&resp_packet_s);

    resp_packet_s.frame_data.cmd_id = START_TRANS_RESP;
    trans_uint16_to_uint8(resp_packet_s.frame_data.data_seg, resp_packet_s.frame_data.data_seg_len);
    send_fb_to_master();
}

uint8_t if_getting_data(void)
{
    return if_get_data;
}
