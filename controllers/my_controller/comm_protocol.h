/**
 ******************************************************************************
 * @file    comm_protocol.h
 * @brief   上位机(PC) ↔ 下位机(MCU) 通信协议定义
 * @note    物理层: UART 115200 bps, 8N1, 小端字节序
 *           帧率:   100 Hz (10ms/帧)
 *           帧长:   固定 60 字节
 *           校验:   CRC-16/MODBUS
 *           本模块仅依赖 head.h (stdint/stddef), 可直接移植到 MCU 工具链
 ******************************************************************************
 */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include "head.h"

/*============================================================================
 * 协议常量
 *============================================================================*/
#define COMM_SOF0 0xAA         /* 帧头字节0 */
#define COMM_SOF1 0x55         /* 帧头字节1 */
#define COMM_FRAME_SIZE 60     /* 帧总长度 (字节) */
#define COMM_PAYLOAD_SIZE 54   /* 载荷长度 (字节) */
#define COMM_HEADER_SIZE 4     /* 帧头长度: SOF0+SOF1+frame_id+seq_num */
#define COMM_CRC_OFFSET 58     /* CRC16 在帧中的偏移 */
#define COMM_CRC_COVERAGE 58   /* CRC 覆盖的字节数 (0~57) */

/* 帧类型 ID */
#define COMM_FRAME_ID_DOWNLINK 0x01 /* PC → MCU: 关节角度 + 控制命令 */
#define COMM_FRAME_ID_UPLINK   0x02 /* MCU → PC: IMU + 足端力 + 状态 */

/* 关节数量 */
#define COMM_NUM_LEGS          4
#define COMM_NUM_JOINTS_PER    3
#define COMM_NUM_JOINTS        12

/*============================================================================
 * 步态模式值 (ctrl_flags0 bits 2-4)
 *============================================================================*/
enum
{
    COMM_GAIT_STAND = 0,
    COMM_GAIT_TROT = 1,
    COMM_GAIT_WALK = 2,
    COMM_GAIT_PRONK = 3
};

/*============================================================================
 * 下行控制标志位掩码 (ctrl_flags0)
 *============================================================================*/
#define COMM_FLAG_EMERGENCY_STOP 0x01  /* 急停: 立即关闭所有电机输出 */
#define COMM_FLAG_MOTOR_ENABLE   0x02  /* 电机使能: 1=上电, 0=去使能 */
#define COMM_FLAG_GAIT_MASK      0x1C  /* 步态模式掩码 (3 bits) */
#define COMM_FLAG_GAIT_SHIFT     2     /* 步态模式左移位数 */
#define COMM_FLAG_JUMP_TRIGGER   0x20  /* 跳跃触发 (边沿: 0→1 触发) */
#define COMM_FLAG_FOLD_TRIGGER   0x40  /* 折叠/展开触发 (边沿: 0→1 触发) */

/*============================================================================
 * 上行状态标志位掩码 (status_flags)
 *============================================================================*/
#define COMM_STAT_FOOT_RF     0x01 /* 右前足触地 */
#define COMM_STAT_FOOT_LF     0x02 /* 左前足触地 */
#define COMM_STAT_FOOT_RB     0x04 /* 右后足触地 */
#define COMM_STAT_FOOT_LB     0x08 /* 左后足触地 */
#define COMM_STAT_IMU_VALID   0x10 /* IMU 数据有效 (已校准) */
#define COMM_STAT_ERROR       0x20 /* 通用错误 (看门狗/电压异常等) */
#define COMM_STAT_MOTOR_FAULT 0x40 /* 电机驱动故障 (过流/过温) */

/*============================================================================
 * 帧载荷结构体
 *============================================================================*/

/* 下行载荷: PC → MCU (54 字节) */
typedef struct __attribute__((packed))
{
    float angles[COMM_NUM_JOINTS]; /* 48B, 关节角度(度), 已方向重映射
                                      [RF_J1,RF_J2,RF_J3, LF_J1,LF_J2,LF_J3,
                                       RB_J1,RB_J2,RB_J3, LB_J1,LB_J2,LB_J3] */
    uint8_t ctrl_flags0;          /* 控制标志字节0: 急停/使能/步态/跳跃/折叠 */
    uint8_t ctrl_flags1;          /* 控制标志字节1: 保留 (置0) */
    uint8_t reserved[4];          /* 保留扩展 (置0) */
} CommDownlinkPayload_t;

/* 上行载荷: MCU → PC (54 字节) */
typedef struct __attribute__((packed))
{
    float imu_euler[3];  /* 12B, 融合后欧拉角 [roll, pitch, yaw], rad */
    float imu_accel[3];  /* 12B, 加速度计 [ax, ay, az], m/s², 机体坐标系 */
    float imu_gyro[3];   /* 12B, 陀螺仪 [gx, gy, gz], rad/s, 机体坐标系 */
    float foot_force[4]; /* 16B, 足端力 [RF, LF, RB, LB], N */
    uint8_t status_flags; /* 状态标志: 触地/IMU有效/错误/电机故障 */
    uint8_t reserved;      /* 保留扩展 (置0) */
} CommUplinkPayload_t;

/*============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief  CRC-16/MODBUS 计算 (逐位算法, 无需查找表)
 * @param  data 数据指针
 * @param  len  数据长度
 * @return CRC16 校验值
 * @note   多项式 0x8005, 初值 0xFFFF, 输入/输出反射
 *         参考值: "123456789" (9字节) → 0x4B37
 */
uint16_t comm_crc16_modbus(const uint8_t *data, size_t len);

/**
 * @brief  校验接收帧 (SOF + CRC)
 * @param  buf 帧缓冲区 (60 字节)
 * @return 0=校验通过, -1=SOF错误, -2=CRC错误
 */
int comm_frame_validate(const uint8_t buf[COMM_FRAME_SIZE]);

/**
 * @brief  打包下行帧 (PC→MCU)
 * @param  buf     输出帧缓冲区 (60 字节)
 * @param  payload 下行载荷指针
 * @param  seq_num 帧序号
 */
void comm_downlink_pack(uint8_t buf[COMM_FRAME_SIZE],
                        const CommDownlinkPayload_t *payload,
                        uint8_t seq_num);

/**
 * @brief  打包上行帧 (MCU→PC)
 * @param  buf     输出帧缓冲区 (60 字节)
 * @param  payload 上行载荷指针
 * @param  seq_num 帧序号
 */
void comm_uplink_pack(uint8_t buf[COMM_FRAME_SIZE],
                      const CommUplinkPayload_t *payload,
                      uint8_t seq_num);

/**
 * @brief  解包下行帧
 * @param  buf         帧缓冲区 (60 字节)
 * @param  payload     输出载荷指针
 * @param  seq_num_out 输出帧序号
 * @return 0=成功, -1=SOF错误, -2=CRC错误
 */
int comm_downlink_unpack(const uint8_t buf[COMM_FRAME_SIZE],
                         CommDownlinkPayload_t *payload,
                         uint8_t *seq_num_out);

/**
 * @brief  解包上行帧
 * @param  buf         帧缓冲区 (60 字节)
 * @param  payload     输出载荷指针
 * @param  seq_num_out 输出帧序号
 * @return 0=成功, -1=SOF错误, -2=CRC错误
 */
int comm_uplink_unpack(const uint8_t buf[COMM_FRAME_SIZE],
                       CommUplinkPayload_t *payload,
                       uint8_t *seq_num_out);

#endif /* COMM_PROTOCOL_H */
