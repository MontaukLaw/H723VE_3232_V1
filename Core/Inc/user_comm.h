#ifndef __USER_COMM_H_
#define __USER_COMM_H_

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "usart.h"
#include "app.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal_dac.h"
#include "stm32h7xx_hal_gpio.h"
#include "spi.h"
#include "ads131.h"
#include "ads131_hal.h"
#include "dac_app.h"
#include "hc4067.h"
#include "comm.h"
#include "algorithm.h"
#include "algorithm_ai.h"
#include "logger.h"
#include "main_app.h"
#include "geer_comm.h"

#define ERROR_FEED_BACK_PACK_LEN 5

#define EMA_DRIFT 0

#define AUTO_AGC_THRESHOLD 5.0f
#define ZERO_INIT_TIMES 30

#define ENABLE_IIR 0
#define IIR_ALPHA 0.1f

#define ADC_SCALE 20

#define MAX_FRAME_DATA_SEG_LEN 7000
#define PER_SENSOR_DATA_LEN 0x06

#define ADC_BUF_SIZE 100
#define UART_TX_BUF_SIZE 100

#define MAX_SAMPLES_TIMES 1
#define MAX_SAPMLES 270
#define DAC_LEN (MAX_SAPMLES * MAX_SAMPLES_TIMES)

#define PI 3.1415926

#define DAC_FLAT_VAL 100
#define DAC_MAX_VALUE 4000 // 4000 // 4095 // 256 // 3000 // 4095 // 1000
#define PERIOD_FLAGS_MAX 8 // 7 // 10
#define HEADROOM 100

#define WAVE_CH_MAX 32 // 波形通道最大值
#define ADC_CH_MAX 32  // ADC 通道最大值
#define TOTAL_POINT_NUMBER (WAVE_CH_MAX * ADC_CH_MAX)
#define TOTAL_SENSOR_NUMBER TOTAL_POINT_NUMBER

#define TARGET_ID 0x78563412
#define DEVICE_ID 0x28041038
#define SENSOR_NUMBER_HEX 0x0004 // 注意大小端 256 0100

#define UART_RX_BUF_LEN 50
#define STANDARD_PROTOCAL_LEN UART_RX_BUF_LEN

#define GEER_UART_TX_BUF_LEN 3000

#define STANDARD_CMD_STATUS_SUCESS 0x0100 // 注意大小端
#define STANDARD_CMD_STATUS_FAIL 0x0000   // 注意大小端

#define CALIB_BASELINE_A 0.25f
#define CALIB_NOISE_A 0.1f

#define IDEL_BASELINE_A 0.05f
#define IDEL_NOISE_A IDEL_BASELINE_A

#define ST_IDLE 0
#define ST_TOUCHING 1

#define INIT_FRAMES 40

#define TOUCHING_THRESHOLD_MULTIPLIER 3.0f

#define FW_VERSION_MAJOR 0x01
#define FW_VERSION_MINOR 0x00
#define FW_VERSION_PATCH 0x00

#define FW_VERSION_PACK_LEN 34

// Header	Length	Type	Module	CMD Type
// 1byte	2byte	1byte	1byte	1byte
// Payload Point*2
// CRC    Tail
// 2byte	1byte
#define GEER_PACK_HEAD 0xA5
#define GEER_PACK_TAIL 0xD5
#define BYTES_PER_POINT 2
#define GEER_DATA_UPLOAD_PACK_LEN (6 + BYTES_PER_POINT * TOTAL_SENSOR_NUMBER + 3)

// 0xFB	无效指令
// 0xFC	数据长度超限
// 0xFD	CRC校验失败
typedef enum
{
    NO_ERROR = 0x00,
    INVALID_CMD = 0xFB,
    DATA_LEN_EXCEED = 0xFC,
    CRC_FAIL = 0xFD,
} GEER_ERROR_CODE;

// 0x0F	软件版本号
// 0X12	压力获取模块
// 0x20	升级模块
typedef enum
{
    TEST_EQUIPMENT_TYPE = 0x0F,
    TEST_EQUIPMENT_MODULE_TYPE = 0x12,
    TEST_EQUIPMENT_UPGRADE_MODULE = 0x20,
} GEER_MODULE_TYPE;

typedef enum
{
    // 查询
    GEER_TYPE_QUERY = 0xFF,

    // 压力模块
    SGCSENSE_PRESSURE_MODULE_FRONT_FACE = 0x20, // front face
    SGCSENSE_PRESSURE_MODULE_BACK_HEAD = 0x21,  // back face
    SGCSENSE_PRESSURE_MODULE_2 = 0x22,
    SGCSENSE_PRESSURE_MODULE_3 = 0x23,
    SGCSENSE_PRESSURE_MODULE_4 = 0x24,
    SGCSENSE_PRESSURE_MODULE_5 = 0x25,
    SGCSENSE_PRESSURE_MODULE_6 = 0x26,
    SGCSENSE_PRESSURE_MODULE_7 = 0x27,
    SGCSENSE_PRESSURE_MODULE_8 = 0x28,
    SGCSENSE_PRESSURE_MODULE_9 = 0x29,

} GEER_EQUP_TYPE;

#define MY_EQUIP_TYPE SGCSENSE_PRESSURE_MODULE_BACK_HEAD

typedef enum
{
    GEER_CMD_READ = 0x01,
    TEST_CMD_MODULE_TYPE = 0x12,
    TEST_CMD_UPGRADE_MODULE = 0x20,
} GEER_CMD_TYPE;

typedef struct geer_protocal
{
    uint16_t payload_length;
    GEER_EQUP_TYPE equipment_type;
    GEER_MODULE_TYPE module;
    GEER_CMD_TYPE cmd_type;
    uint8_t payload[UART_RX_BUF_LEN - 8];
    uint16_t crc;
} geer_protocal_t;

typedef struct g_point
{
    uint16_t adc_idx;
    uint16_t wave_idx;
} g_point_t;

typedef struct g_ver_t
{
    bool initialized;
    float baseline[TOTAL_SENSOR_NUMBER];
    float base_noise[TOTAL_SENSOR_NUMBER];

    uint32_t baseline_u32[TOTAL_SENSOR_NUMBER];
    uint32_t basenoise_u32[TOTAL_SENSOR_NUMBER];
    uint8_t state;

} g_ver;

typedef enum
{
    QUERY_DEVICE_ID = 0x0001,
    STOP_TRANS = 0x0013,
    START_TRANS = 0x0011,
    INIT_DEVICE = 0x0003,
    SYNC_DATA_TYPE = 0x0015,
} STD_CMD_TYPE;

typedef enum
{
    QUERY_DEVICE_ID_RESP = 0x0200, // 注意大小端
    STOP_TRANS_RESP = 0x1400,
    START_TRANS_RESP = 0x1200,
    INIT_DEVICE_RESP = 0x0400,
    // INIT_DEVICE_RESP = 0x0600,
    SYNC_DATA_TYPE_RESP = 0x1600,
} STD_RESP_TYPE;

typedef struct g_response_frame_t
{
    uint32_t device_id;
    uint32_t target_id;
    uint16_t cmd_id;
    uint16_t status_id;
    uint8_t data_seg[MAX_FRAME_DATA_SEG_LEN];
    // 数据帧仅到这里
    // 这里仅仅记录data_seg的长度
    uint16_t data_seg_len;
} response_frame_t;

typedef struct g_response_packet_t
{
    uint8_t head[4];
    uint16_t frame_id;
    uint16_t frame_len;
    response_frame_t frame_data;
    uint16_t whole_packet_len; // 用于统计发送长度
} response_packet_t;

extern uint8_t uart1_rx_buf[];

#endif
