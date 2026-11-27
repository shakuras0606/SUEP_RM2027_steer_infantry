#ifndef SHOOT_H
#define SHOOT_H

/* 摩擦轮: DJI M3508 + C620 电调, 节点号 1/2, 挂载在 CAN2 */
#define SHOOT_MOTOR_CAN         (&hcan2)
#define SHOOT_FRICTION_L_CAN_ID 1u
#define SHOOT_FRICTION_R_CAN_ID 2u

/* 摩擦轮转速参考值,单位:度/秒(DJIMotor速度环单位)。
 * 20m/s左右弹速对应的轮速可按 轮速(rpm)=弹速/(pi*摩擦轮直径)*60 估算,
 * 下面的数值是占位值,必须实测弹道后重新标定 */
#define SHOOT_FRICTION_SPEED_15 30000.0f // ≈5000rpm
#define SHOOT_FRICTION_SPEED_18 36000.0f // ≈6000rpm
#define SHOOT_FRICTION_SPEED_30 45000.0f // ≈7500rpm
#define SHOOT_FRICTION_SPEED_DEFAULT 30000.0f // 未收到有效弹速时的调试默认值

/* 拨弹轮: 达妙 DM4310, MIT 控制模式, 挂载在 CAN2
 * MIT 模式控制帧 = 0x300 + ESC_ID, 反馈帧 = MST_ID。
 * 需在达妙调试助手中配置: ESC_ID=5(控制帧0x305), MST_ID=0x015 或 0x005
 * (反馈报文 D[0] 低4位为电机ID, 必须与 ESC_ID 的低4位一致才能被驱动解析) */
#define SHOOT_LOADER_DM_ESC_ID  5u
#define SHOOT_LOADER_DM_CTRL_ID (0x300u + SHOOT_LOADER_DM_ESC_ID) // 0x305
#define SHOOT_LOADER_DM_MST_ID  0x015u                            // 反馈帧ID

/* MIT 模式量程,必须与达妙调试助手中写入电机的 PMAX/VMAX/TMAX 一致,
 * 否则位置/速度/力矩的换算会整体缩放错误(底盘转向电机同样依赖该组参数) */
#define LOADER_PMAX 12.5f // rad
#define LOADER_VMAX 30.0f // rad/s
#define LOADER_TMAX 10.0f // N·m

/* MIT 位置/速度内部增益(电机固件内实现,不需要主控再跑PID) */
#define LOADER_POS_KP   8.0f // 拨弹定位刚度
#define LOADER_POS_KD   0.5f // 拨弹定位阻尼
#define LOADER_SPEED_KD 0.3f // 连发/反转的速度跟踪阻尼

/* 拨盘反转角速度,rad/s */
#define LOADER_REVERSE_SPEED 5.0f

/* 机构参数 */
#define ONE_BULLET_DELTA_ANGLE 36.0f // 发射一发弹丸拨盘转动的角度(电机输出轴),由机械设计图纸给出
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量（预留）

/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

#endif // SHOOT_H
