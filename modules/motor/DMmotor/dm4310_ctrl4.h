/**
 * @file dm4310_ctrl4.h
 * @brief 达妙 DM4310 一控四(V41)固件电机驱动
 *
 * @details
 *  一控四固件会自动把电机设置为电流环模式,不再接受 MIT/位置速度/力位混控报文,
 *  而是用一帧 8 字节控制报文同时控制 4 个电机:
 *    - 电机 ID 1~4: 控制帧 0x3FE, D[2k]/D[2k+1] 为第 k 个电机(ID=k+1)的电流低/高 8 位;
 *    - 电机 ID 5~8: 控制帧 0x4FE, 数据段排布相同。
 *  控制电流为标幺值,放大 10000 倍,即 10000 对应电机的最大电流 Imax(上电打印值)。
 *  0x3FE/0x4FE 两个控制帧内部按 4 个 slot 排布,因此本驱动按"控制组"聚合发送。
 *
 *  反馈帧: 电机 1ms 主动上传,ID=0x300+电机ID,8 字节:
 *    D[0]/D[1] 位置(高/低 8 位, 0~8191 对应一圈)
 *    D[2]/D[3] 速度(高/低 8 位, 单位 rpm×100)
 *    D[4]/D[5] 扭矩电流(高/低 8 位, 单位 mA)
 *    D[6]      线圈温度 ℃
 *    D[7]      错误状态
 *
 *  本模块在电流环外面用软件实现了角度环/速度环串级,用法与 dji_motor 一致,
 *  应用层把电机当作增益为 1 的执行器,直接 DJIMotorSetRef 式的设置参考值即可。
 *
 * @note 1. 初始化时 Motor_Init_Config_s.can_init_config.tx_id 传入"电机ID"(1~8),
 *          驱动内部会把它改写为控制组帧 ID(0x3FE/0x4FE) 并自动设置反馈 ID(0x300+ID)。
 *       2. 一控四固件反馈中的电流为有符号量(mA),控制电流按有符号 16 位补码发送,
 *          方向为上电时认定的正方向;若实测方向相反,把 motor_reverse_flag 设为
 *          MOTOR_DIRECTION_REVERSE 或调换电机相序。
 *       3. 同一控制组(0x3FE 或 0x4FE)的电机必须挂载在同一条 CAN 总线上。
 *       4. 电机波特率/控制组 ID 需要在达妙调试助手或寄存器中配置好,本驱动不负责配置。
 *
 * @version 1.0
 * @date 2026-11-26
 */
#ifndef DM4310_CTRL4_H
#define DM4310_CTRL4_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#include "stdint.h"

#define DM4310_CTRL4_MOTOR_CNT 8 // 一控四最多支持8个电机(2个控制组)

/* 一控四控制帧 ID 与反馈帧 ID */
#define DM4310_CTRL4_GROUP_LOW_CAN_ID 0x3FEu  // 电机ID 1~4
#define DM4310_CTRL4_GROUP_HIGH_CAN_ID 0x4FEu // 电机ID 5~8
#define DM4310_CTRL4_FEEDBACK_ID_BASE 0x300u  // 反馈帧 ID = 0x300 + 电机ID

/* 控制电流标幺值上限,10000 对应电机最大电流 Imax */
#define DM4310_CTRL4_CURRENT_MAX 10000.0f

/* 反馈量转换系数 */
#define DM4310_CTRL4_ECD_ANGLE_COEF 0.043945f  // 360/8192,编码器值→度
#define DM4310_CTRL4_SPEED_2_ANGLE_PER_SEC 0.06f // rpm×100 → 度/秒 (×6/100)
#define DM4310_CTRL4_SPEED_SMOOTH_COEF 0.85f   // 速度低通滤波系数,1为关闭滤波
#define DM4310_CTRL4_CURRENT_SMOOTH_COEF 0.9f  // 电流低通滤波系数,1为关闭滤波

/* DM4310 一控四反馈信息 */
typedef struct
{
    uint16_t last_ecd;        // 上一次的编码器值
    uint16_t ecd;             // 0~8191,一圈8192格
    float angle_single_round; // 单圈角度,单位:度
    float speed_aps;          // 角速度,单位:度/秒
    float current_mA;         // 扭矩电流,单位:mA
    uint8_t temperature;      // 线圈温度,单位:℃
    uint8_t error_state;      // 错误状态,详见达妙错误状态说明书
    float total_angle;        // 多圈总角度,单位:度
    int32_t total_round;      // 多圈圈数
    uint32_t updated_ms;      // 最近一次收到反馈的时间戳(ms)
    uint8_t valid;            // 是否收到过有效反馈
} DM4310Ctrl4_Measure_s;

typedef struct
{
    DM4310Ctrl4_Measure_s measure;          // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置(反馈来源/正反转/闭环类型)
    Motor_Controller_s motor_controller;    // 电机控制器(三环PID与参考值)

    CANInstance *motor_can_instance; // 本电机反馈用的 CAN 实例
    CANInstance *tx_sender;          // 所属控制组的发送实例(组内第一个电机拥有)
    uint8_t motor_id;                // 电机ID 1~8
    uint8_t sender_group;            // 控制组下标: 0=0x3FE, 1=0x4FE
    uint8_t message_num;             // 组内 slot 下标 0~3

    Motor_Working_Type_e stop_flag; // 启停标志
    DaemonInstance *daemon;         // 离线检测
    uint32_t feed_cnt;              // 反馈周期统计
    float dt;                       // 反馈周期,单位:秒
} DM4310Ctrl4Instance;

/**
 * @brief 注册一个 DM4310 一控四电机,应在应用初始化阶段调用
 *
 * @param config 电机初始化配置,复用 DJI 电机的 Motor_Init_Config_s。
 *               can_init_config.tx_id 传入电机ID(1~8);can_handle 为挂载总线。
 *               controller_param_init_config 中配置三环 PID 及额外反馈指针,
 *               controller_setting_init_config 中配置闭环类型/反馈来源/正反转。
 * @return DM4310Ctrl4Instance* 电机实例指针,失败返回 NULL
 */
DM4310Ctrl4Instance *DM4310Ctrl4Init(Motor_Init_Config_s *config);

/**
 * @brief 设定电机参考值(角度/速度/电流,取决于 outer_loop_type)
 *
 * @param motor 电机实例
 * @param ref   参考值。角度环/速度环参考值单位为度、度/秒;开环时为单位为标幺电流。
 */
void DM4310Ctrl4SetRef(DM4310Ctrl4Instance *motor, float ref);

/**
 * @brief 切换某个闭环的反馈来源(如把角度/速度环切到 IMU)
 *
 * @param motor 电机实例
 * @param loop  要切换的闭环(ANGLE_LOOP 或 SPEED_LOOP)
 * @param type  反馈来源(MOTOR_FEED / OTHER_FEED)
 */
void DM4310Ctrl4ChangeFeed(DM4310Ctrl4Instance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @brief 修改电机外层闭环类型
 *
 * @param motor      电机实例
 * @param outer_loop 外层闭环类型
 */
void DM4310Ctrl4OuterLoop(DM4310Ctrl4Instance *motor, Closeloop_Type_e outer_loop);

/**
 * @brief 停止电机: 发送给该控制组的电流指令会被置为0
 *
 * @param motor 电机实例
 */
void DM4310Ctrl4Stop(DM4310Ctrl4Instance *motor);

/**
 * @brief 启动电机,使其响应参考值
 *
 * @param motor 电机实例
 */
void DM4310Ctrl4Enable(DM4310Ctrl4Instance *motor);

/**
 * @brief 被 motor_task 调用,计算所有实例的串级 PID,并按控制组发送电流帧
 *
 * @note 每个控制组每调用一次只发送一帧 CAN 报文,4个电机的电流在帧内并行下发。
 */
void DM4310Ctrl4Control(void);

#endif // !DM4310_CTRL4_H
