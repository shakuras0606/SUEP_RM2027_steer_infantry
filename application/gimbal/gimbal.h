#ifndef GIMBAL_H
#define GIMBAL_H

/* 云台电机接线（达妙 DM4310 一控四固件 V41）
 * - 电机ID 1~4 共用控制帧 0x3FE, 反馈帧为 0x300+电机ID, 故 yaw/pitch 必须配置为 1、2 号
 * - 一控四固件为电流环模式, 角度/速度串级在 dm4310_ctrl4 模块中软件实现
 * - 电机ID、控制组、波特率需先在达妙调试助手中配置好, 本工程不负责写电机参数 */
#define GIMBAL_MOTOR_CAN     (&hcan3)
#define GIMBAL_YAW_MOTOR_ID  1u // 云台yaw电机在达妙工具中配置的电机ID
#define GIMBAL_PITCH_MOTOR_ID 2u // 云台pitch电机在达妙工具中配置的电机ID

/* 云台安装标定
 * @note 一控四固件使用单圈绝对位置编码器, 设置零点(0x7FF 0x55 0x50)后编码器值会改变,
 *       因此刷写/重新标定零点后必须重新测量下面的对齐编码器值 */
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度

/* 预留标定项，当前控制逻辑尚未使用 */
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向,1为相同,-1为相反

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H
