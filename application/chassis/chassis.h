#ifndef CHASSIS_H
#define CHASSIS_H

/* ==================== 电机 CAN 配置 ====================
 * 舵电机: 达妙 DM4310, MIT 控制模式, 挂载 CAN2
 *   - 控制帧 = CAN_ID(ESC_ID), 反馈帧 = MST_ID
 *   - 反馈报文 D[0] 低4位为电机ID, 必须与 CAN_ID 的低4位一致, 否则反馈被整帧丢弃
 * 驱动电机: 领控 LK7015, 挂载 CAN1, 反馈帧 = 0x140 + 节点号(节点号必须 1~4)
 * 电机侧的 CAN_ID/MST_ID/节点号需先用达妙调试助手、LK 上位机配置好, 本工程不负责写电机参数 */
#define CHASSIS_STEER_CAN (&hcan2)
#define CHASSIS_DRIVE_CAN (&hcan1)

#define CHASSIS_STEER_FL_CAN_ID 0x001u // 左前 舵电机 MIT CAN_ID
#define CHASSIS_STEER_FL_MST_ID 0x011u // 左前 舵电机反馈 MST_ID
#define CHASSIS_DRIVE_FL_NODE   1u     // 左前 LK7015 节点号

#define CHASSIS_STEER_FR_CAN_ID 0x002u // 右前 舵电机 MIT CAN_ID
#define CHASSIS_STEER_FR_MST_ID 0x012u // 右前 舵电机反馈 MST_ID
#define CHASSIS_DRIVE_FR_NODE   2u     // 右前 LK7015 节点号

#define CHASSIS_STEER_RL_CAN_ID 0x003u // 左后 舵电机 MIT CAN_ID
#define CHASSIS_STEER_RL_MST_ID 0x013u // 左后 舵电机反馈 MST_ID
#define CHASSIS_DRIVE_RL_NODE   3u     // 左后 LK7015 节点号

#define CHASSIS_STEER_RR_CAN_ID 0x004u // 右后 舵电机 MIT CAN_ID
#define CHASSIS_STEER_RR_MST_ID 0x014u // 右后 舵电机反馈 MST_ID
#define CHASSIS_DRIVE_RR_NODE   4u     // 右后 LK7015 节点号

/* 编译期校验: MST_ID 低4位必须等于 CAN_ID 低4位, 改ID时漏改一侧会在这里直接报错 */
#if (((CHASSIS_STEER_FL_MST_ID ^ CHASSIS_STEER_FL_CAN_ID) | \
      (CHASSIS_STEER_FR_MST_ID ^ CHASSIS_STEER_FR_CAN_ID) | \
      (CHASSIS_STEER_RL_MST_ID ^ CHASSIS_STEER_RL_CAN_ID) | \
      (CHASSIS_STEER_RR_MST_ID ^ CHASSIS_STEER_RR_CAN_ID)) & 0x0fu)
#error "CHASSIS_STEER_x_MST_ID 的低4位必须等于对应 CHASSIS_STEER_x_CAN_ID 的低4位"
#endif

/* 底盘速度、跟随控制和舵电机 MIT 参数 */
#define SW_VXY       0.40f   // 遥控平移上限，m/s
#define SW_WZ        1.00f   // 跟随角速度上限，rad/s
#define SW_FOLLOW_KP 0.03f   // (rad/s)/deg
#define SW_ROTATE_WZ 1.50f   // rad/s
#define SW_VWHEEL    0.60f   // 轮缘速度上限，m/s
#define SW_PMAX      12.5f   // MIT 参数 PMAX，rad
#define SW_VMAX      30.0f   // VMAX，rad/s
#define SW_TMAX      10.0f   // TMAX，N·m
#define SW_KP        3.0f
#define SW_KD        0.35f

/* 底盘几何尺寸（mm）和传动比 */
#define WHEEL_BASE 350              // 纵向轴距(前进后退方向)
#define TRACK_WIDTH 300             // 横向轮距(左右平移方向)
#define CENTER_GIMBAL_OFFSET_X 0    // 云台中心前后偏移（预留）
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台中心左右偏移（预留）
#define RADIUS_WHEEL 60             // 轮子半径
#define REDUCTION_RATIO_WHEEL 19.0f // 轮电机减速比（当前轮毂驱动未使用）

/* 各轮朝前时的舵电机输出轴角度（rad） */
#define CHASSIS_FL_ZERO_RAD 0.0f  // 左前
#define CHASSIS_FR_ZERO_RAD 0.0f  // 右前
#define CHASSIS_RL_ZERO_RAD 0.0f  // 左后
#define CHASSIS_RR_ZERO_RAD 0.0f  // 右后

#define CHASSIS_STEER_HOLD_MPS   0.01f  // 轮速低于此值时保持上次舵角
#define CHASSIS_STEER_MARGIN_RAD 0.05f  // 舵电机位置量程两端的预留量

/* 底盘控制时序（ms） */
#define CHASSIS_CONTROL_PERIOD_MS   5u
#define CHASSIS_FEEDBACK_TIMEOUT_MS 60u   // 指令和电机反馈共用，也用作停机重发间隔
#define CHASSIS_START_TIMEOUT_MS    500u  // 等待全部轮组就绪的最长时间

/**
 * @brief 底盘应用初始化,请在开启rtos之前调用(目前会被RobotInit()调用)
 * 
 */
void ChassisInit(void);

/**
 * @brief 底盘应用任务,放入实时系统以一定频率运行
 * 
 */
void ChassisTask(void);

/* 从电机任务调用，舵轮的唯一输出入口。 */
void ChassisMotorTask(void);

#endif // CHASSIS_H