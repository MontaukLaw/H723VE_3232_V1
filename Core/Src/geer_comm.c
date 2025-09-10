#include "user_comm.h"

geer_protocal_t recv_packet;
const uint8_t complie_time[] = {0x25, 0x09, 0x07, 0x12, 0x00, 0x00}; // 编译时间

__IO uint16_t upload_period = 0; // 数据上报周期，单位ms

__attribute__((section("dma_buffer"), aligned(32)))
uint8_t uart1_tx_buf[GEER_UART_TX_BUF_LEN];

GEER_ERROR_CODE packaget_legal_check(uint16_t len, uint8_t *data)
{

    if (data[0] != 0xA5 || data[len - 1] != 0xD5)
    {
        return INVALID_CMD;
    }
    memset(&recv_packet, 0, sizeof(geer_protocal_t));
    // length=type+module+cmd type+payload+crc+tail
    recv_packet.payload_length = ((uint16_t)data[1]) << 8 | data[2];
    // 判断length是否正确
    if (recv_packet.payload_length + 3 != len)
    {
        return DATA_LEN_EXCEED;
    }

    // check crc
    uint16_t recv_crc = ((uint16_t)data[len - 3]) << 8 | data[len - 2];

    uint16_t calc_crc = checksum_crc16(data, len - 3);

    if (recv_crc != calc_crc)
    {
        return CRC_FAIL;
    }

    recv_packet.equipment_type = data[3];
    recv_packet.module = data[4];
    recv_packet.cmd_type = data[5];

    if (recv_packet.payload_length > 6)
    {
        memcpy(recv_packet.payload, &data[6], recv_packet.payload_length - 6);
    }

    return NO_ERROR;
}

// Header	Length	ERROR Code	Tail
// 1byte	2byte	  1byte	1byte
// 0xF5   0x01  	Error code	0xD5
// 小端在前
void feed_error_back(GEER_ERROR_CODE err_code)
{

    // memset(uart1_rx_buf, 0, sizeof(uart1_rx_buf));
    memset(uart1_tx_buf, 0, sizeof(uint8_t) * GEER_UART_TX_BUF_LEN);

    uart1_tx_buf[0] = 0xF5;
    uart1_tx_buf[1] = 0x01;
    uart1_tx_buf[2] = 0x00;
    uart1_tx_buf[3] = err_code;
    uart1_tx_buf[4] = 0xD5;

    HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, ERROR_FEED_BACK_PACK_LEN);
}

// Header	Length	Type	Module	CMD Type	Payload	CRC	Tail
// 1byte	2byte	1byte	1byte	1byte	N byte	2byte	1byte
// 0xA5	"length=type+module+cmd type+payload+crc+tail
// if payload is Null ,length=6"	test equipment type	test equipment module type	command type+0x80		CRC16 except tail	0xD5
void feed_back(uint16_t len, uint8_t *data)
{
}

// 0xA5	L|H	test equipment type(实际设备类型)
// 0x0F 0x81	Major Minor	Patch 编译时间
// UID	区域类型 size类型 CRCL|CRCH	0xD5
void send_fw_version(void)
{

    uint8_t i;
    memset(uart1_tx_buf, 0, sizeof(uint8_t) * FW_VERSION_PACK_LEN);

    uart1_tx_buf[0] = 0xA5;

    // length
    // 长度等于除去tail之外的所有字节
    uint16_t payload_len = FW_VERSION_PACK_LEN - 3;
    uart1_tx_buf[1] = payload_len & 0x00FF;
    uart1_tx_buf[2] = (payload_len & 0xFF00) >> 8;

    // type // 这个是前脸
    uart1_tx_buf[3] = MY_EQUIP_TYPE;

    // module
    // 这里客户要求是0x0F
    uart1_tx_buf[4] = TEST_EQUIPMENT_TYPE;

    // cmd type
    uart1_tx_buf[5] = GEER_CMD_READ + 0x80;

    // payload
    // Major
    uart1_tx_buf[6] = FW_VERSION_MAJOR;
    // Minor
    uart1_tx_buf[7] = FW_VERSION_MINOR;
    // Patch
    uart1_tx_buf[8] = FW_VERSION_PATCH;

    // 编译时间
    // 6Byte
    for (i = 0; i < 6; ++i)
        uart1_tx_buf[9 + i] = complie_time[i];
    // memcpy(&uart1_tx_buf[9], (uint8_t *)complie_time, 6);

    // UID 12Byte
    // uint32_t *uid = (uint32_t *)UID_BASE;
    // memcpy(&uart1_tx_buf[15], uid, 12);
    const uint8_t *src = (const uint8_t *)UID_BASE;
    for (i = 0; i < 12; ++i)
    {
        uart1_tx_buf[15 + i] = src[i];
    }

    // 区域类型
    // 欧洲 EU 0x45 0x55
    // 亚洲	AS 0x41 0x53
    uart1_tx_buf[27] = 0x41;
    uart1_tx_buf[28] = 0x53;

    // L	0X30 0X4C
    // M	0X30 0X4D
    // S	0X30 0X53
    uart1_tx_buf[29] = 0x30;
    uart1_tx_buf[30] = 0x4D;

    // crc
    // uint16_t crc = checksum_crc16(&uart1_rx_buf[1], len - 3);
    uint16_t crc = checksum_crc16(&uart1_tx_buf[0], FW_VERSION_PACK_LEN - 3);

    uart1_tx_buf[31] = (crc & 0x00FF);
    uart1_tx_buf[32] = (crc & 0xFF00) >> 8;

    // tail
    uart1_tx_buf[33] = 0xD5;

    HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, FW_VERSION_PACK_LEN);
}

void uart_test(void)
{
    send_fw_version();
    // static uint8_t print_counter = 0;
    // print_counter++;
    // memset(uart1_tx_buf, 0xcc, sizeof(uint8_t) * FW_VERSION_PACK_LEN);
    // uart1_tx_buf[0] = print_counter;
    // HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, FW_VERSION_PACK_LEN);
}

// Header	Length	Type	Module	CMD Type	Payload	CRC	Tail
// 1byte	2byte	1byte	1byte	1byte	N byte	2byte	1byte
// 0xA5     0xD5
// "length=type+module+cmd type+payload+crc+tail if payload is Null ,length=6"
// test equipment type
// test equipment module type
// command type
void geer_comm_handler(void)
{

    if (got_rx)
    {
        GEER_ERROR_CODE err = packaget_legal_check(got_rx, uart1_rx_buf);
        if (err == NO_ERROR)
        {
            // 查询版本号
            if (recv_packet.equipment_type == GEER_TYPE_QUERY &&
                recv_packet.cmd_type == GEER_CMD_READ &&
                recv_packet.module == TEST_EQUIPMENT_TYPE)
            {
                printf("Checking SW\r\n");
                send_fw_version();
            }
            // 0xA5	H|L	test_equipment type	0x12 0x01 数据上报周期ms	CRCH	CRCL	0xD5
            else if (recv_packet.equipment_type == MY_EQUIP_TYPE &&
                     recv_packet.module == TEST_EQUIPMENT_MODULE_TYPE &&
                     recv_packet.cmd_type == GEER_CMD_READ)
            {
                // 设置数据上报周期
                upload_period = ((uint16_t)recv_packet.payload[0] << 8) | (((uint16_t)recv_packet.payload[1]));
                // 如果为零, 就表示不上报
                printf("Set upload period to %u ms\r\n", upload_period);
                // 不用发送反馈
            }
        }
        else
        {
            // 发送反馈
        }
        got_rx = 0;
    }
}

const static uint16_t crc16tab[256] =
    {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
        0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
        0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
        0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
        0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
        0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
        0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
        0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
        0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
        0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
        0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
        0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
        0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
        0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
        0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
        0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
        0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
        0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
        0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
        0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
        0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
        0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

uint16_t checksum_crc16(uint8_t const *ptr, uint16_t len)
{
    if (!ptr)
        return 0x00;

    uint16_t crc = 0;
    uint16_t counter = 0;

    for (counter = 0; counter < len; counter++)
    {
        crc = (crc << 8) ^ crc16tab[((crc >> 8) ^ (*(char *)ptr++)) & 0x00FF];
    }

    return crc;
}

// 1byte 2byte 1byte 1byte 1byte Xbyte * N压力点数
// 0xA5	L|H	test equipment type	0x12 0x81 (N * 原始压力值（单个占用X字节）)
// 每个数据是3字节
// 还未加入真实数据
void data_upload_handle(void)
{
    static uint32_t count = 0;
    static uint32_t tick = 0;
    uint16_t i;
    uint16_t idx = 0;

    if (upload_period == 0)
    {
        return;
    }

    if (HAL_GetTick() - tick < upload_period)
        return;

    tick = HAL_GetTick();
    memset(uart1_tx_buf, 0, sizeof(uint8_t) * GEER_DATA_UPLOAD_PACK_LEN);
    uart1_tx_buf[0] = GEER_PACK_HEAD;

    // length
    // Type + Module + CMD Type + Payload + CRC + Tail
    // payload长度 = type+module+cmd type+crc+tail
    uint16_t pay_load_len = 3 + TOTAL_SENSOR_NUMBER * BYTES_PER_POINT + 3;
    uart1_tx_buf[1] = pay_load_len & 0x00FF;
    uart1_tx_buf[2] = (pay_load_len & 0xFF00) >> 8;

    // type
    uart1_tx_buf[3] = MY_EQUIP_TYPE;

    // module
    uart1_tx_buf[4] = TEST_EQUIPMENT_MODULE_TYPE;

    // cmd type
    uart1_tx_buf[5] = GEER_CMD_READ | 0x80;

    // payload
    // N*原始压力值（单个占用X字节）
    // 模拟数据
    for (i = 0; i < TOTAL_SENSOR_NUMBER; i++)
    {
        uart1_tx_buf[6 + idx] = adc_result_16bit[i] & 0xFF;
        uart1_tx_buf[7 + idx] = (adc_result_16bit[i] & 0xFF00) >> 8;
        idx = idx + 2;
    }

    // crc
    // uint16_t crc = checksum_crc16(&uart1_rx_buf[1], len - 3);
    uint16_t crc = checksum_crc16(uart1_tx_buf, GEER_DATA_UPLOAD_PACK_LEN - 3);

    uart1_tx_buf[GEER_DATA_UPLOAD_PACK_LEN - 3] = (crc & 0x00FF);
    uart1_tx_buf[GEER_DATA_UPLOAD_PACK_LEN - 2] = (crc & 0xFF00) >> 8;

    // tail
    uart1_tx_buf[GEER_DATA_UPLOAD_PACK_LEN - 1] = 0xD5;

    HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, GEER_DATA_UPLOAD_PACK_LEN);
}