#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

/* 底盘输入映射 */
#define ROBOT_CMD_STICK_DEADBAND 20   // 摇杆原始计数，绝对值小于此值时归零
#define ROBOT_CMD_ZERO_STICK 600     // 内八四轴的端点阈值
#define ROBOT_CMD_ZERO_HOLD_MS 1000u
#define ROBOT_CMD_KEY_SPEED      300  // 键盘平移指令系数；当前模式逻辑在键鼠档将底盘速度清零

/* 输入串口 */
#define ROBOT_CMD_REMOTE_UART (&huart5)
#define ROBOT_CMD_VISION_UART (&huart9)

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H
