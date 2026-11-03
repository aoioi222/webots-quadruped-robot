/**
 ******************************************************************************
 * @file    comm_protocol.c
 * @brief   通信协议实现: CRC-16/MODBUS, 帧打包/解包, 帧校验
 ******************************************************************************
 */

#include "comm_protocol.h"
#include <string.h>

/*============================================================================
 * 编译期静态断言 — 确保载荷结构体大小与协议定义一致
 *============================================================================*/
/* C99 兼容: 条件为假时数组大小为负, 触发编译错误 */
#define COMM_STATIC_ASSERT(cond, tag) \
    typedef char tag[(cond) ? 1 : -1]

COMM_STATIC_ASSERT(sizeof(CommDownlinkPayload_t) == COMM_PAYLOAD_SIZE,
                   downlink_payload_size_check);
COMM_STATIC_ASSERT(sizeof(CommUplinkPayload_t) == COMM_PAYLOAD_SIZE,
                   uplink_payload_size_check);

/*============================================================================
 * CRC-16/MODBUS
 *============================================================================*/

/**
 * @brief  CRC-16/MODBUS 计算
 * @param  data 数据指针
 * @param  len  数据长度 (字节)
 * @return CRC16 校验值
 * @note   多项式: 0x8005 (x^16 + x^15 + x^2 + 1)
 *         初值:   0xFFFF
 *         输入/输出均反射 (reflected)
 *         参考值: 字符串 "123456789" → 0x4B37
 */
uint16_t comm_crc16_modbus(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001; /* 0xA001 = 0x8005 反射 */
            else
                crc >>= 1;
        }
    }
    return crc;
}

/*============================================================================
 * 帧校验
 *============================================================================*/

/**
 * @brief  校验接收帧的 SOF 和 CRC
 * @param  buf 帧缓冲区 (60 字节)
 * @return 0=通过, -1=SOF错误, -2=CRC错误
 */
int comm_frame_validate(const uint8_t buf[COMM_FRAME_SIZE])
{
    /* 校验帧头 */
    if (buf[0] != COMM_SOF0 || buf[1] != COMM_SOF1)
        return -1;

    /* 校验 CRC: 覆盖字节 0~57 */
    uint16_t expected = comm_crc16_modbus(buf, COMM_CRC_COVERAGE);
    uint16_t received = (uint16_t)buf[COMM_CRC_OFFSET] |
                        ((uint16_t)buf[COMM_CRC_OFFSET + 1] << 8);

    if (expected != received)
        return -2;

    return 0;
}

/*============================================================================
 * 帧打包
 *============================================================================*/

/**
 * @brief  将载荷写入帧头, 清零保留字段, 计算并填入 CRC
 * @param  buf     输出帧缓冲区
 * @param  frame_id 帧类型 ID
 * @param  payload 载荷数据指针
 * @param  payload_size 载荷大小 (应为 COMM_PAYLOAD_SIZE)
 * @param  seq_num 帧序号
 */
static void comm_pack_frame(uint8_t buf[COMM_FRAME_SIZE],
                            uint8_t frame_id,
                            const void *payload,
                            size_t payload_size,
                            uint8_t seq_num)
{
    /* 帧头 */
    buf[0] = COMM_SOF0;
    buf[1] = COMM_SOF1;
    buf[2] = frame_id;
    buf[3] = seq_num;

    /* 载荷 (memcpy 处理可能的对齐问题) */
    memcpy(&buf[COMM_HEADER_SIZE], payload, payload_size);

    /* 计算并填入 CRC (小端序: 低字节在前) */
    uint16_t crc = comm_crc16_modbus(buf, COMM_CRC_COVERAGE);
    buf[COMM_CRC_OFFSET] = (uint8_t)(crc & 0xFF);
    buf[COMM_CRC_OFFSET + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

void comm_downlink_pack(uint8_t buf[COMM_FRAME_SIZE],
                        const CommDownlinkPayload_t *payload,
                        uint8_t seq_num)
{
    comm_pack_frame(buf, COMM_FRAME_ID_DOWNLINK,
                    payload, sizeof(CommDownlinkPayload_t), seq_num);
}

void comm_uplink_pack(uint8_t buf[COMM_FRAME_SIZE],
                      const CommUplinkPayload_t *payload,
                      uint8_t seq_num)
{
    comm_pack_frame(buf, COMM_FRAME_ID_UPLINK,
                    payload, sizeof(CommUplinkPayload_t), seq_num);
}

/*============================================================================
 * 帧解包
 *============================================================================*/

/**
 * @brief  校验帧, 通过后提取载荷和序号
 * @param  buf         帧缓冲区
 * @param  payload     输出载荷指针
 * @param  payload_size 载荷大小
 * @param  seq_num_out 输出帧序号
 * @return 0=成功, 负值=失败 (同 comm_frame_validate)
 */
static int comm_unpack_frame(const uint8_t buf[COMM_FRAME_SIZE],
                             void *payload,
                             size_t payload_size,
                             uint8_t *seq_num_out)
{
    int ret = comm_frame_validate(buf);
    if (ret != 0)
        return ret;

    if (seq_num_out != NULL)
        *seq_num_out = buf[3];

    memcpy(payload, &buf[COMM_HEADER_SIZE], payload_size);
    return 0;
}

int comm_downlink_unpack(const uint8_t buf[COMM_FRAME_SIZE],
                         CommDownlinkPayload_t *payload,
                         uint8_t *seq_num_out)
{
    return comm_unpack_frame(buf, payload,
                             sizeof(CommDownlinkPayload_t), seq_num_out);
}

int comm_uplink_unpack(const uint8_t buf[COMM_FRAME_SIZE],
                       CommUplinkPayload_t *payload,
                       uint8_t *seq_num_out)
{
    return comm_unpack_frame(buf, payload,
                             sizeof(CommUplinkPayload_t), seq_num_out);
}
