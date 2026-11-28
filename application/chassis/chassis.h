#ifndef CHASSIS_H
#define CHASSIS_H

/* ==================== 电机 CAN 配置 ====================
 * 轮位按顺时针编号: 左前1 → 右前2 → 右后3 → 左后4, 与 steering.h 的 WheelIndex_e 顺序一致,
 * 因此 debug 里 wheels[0..3] / motors[1..4] 依次就是 FL/FR/RR/RL
 * 舵电机: 达妙 DM4310, MIT 控制模式, 挂载 CAN2
 *   - 控制帧 = CAN_ID(ESC_ID), 反馈帧 = MST_ID
 *   - 反馈报文 D[0] 低4位为电机ID, 必须与 CAN_ID 的低4位一致, 否则反馈被整帧丢弃
 * 驱动电机: 领控 LK7015, 挂载 CAN1, 反馈帧 = 0x140 + 节点号(节点号必须 1~4)
 * 电机侧的 CAN_ID/MST_ID/节点号需先用达妙调试助手、LK 上位机配置好, 本工程不负责写电机参数 */
#define CHASSIS_STEER_CAN (&hcan2)
#define CHASSIS_DRIVE_CAN (&hcan1)

/* 顺时针1: 左前 */
#define CHASSIS_STEER_FL_CAN_ID 0x51u // 舵电机 MIT CAN_ID(控制帧)
#define CHASSIS_STEER_FL_MST_ID 0x01u // 舵电机反馈帧 MST_ID
#define CHASSIS_DRIVE_FL_NODE   1u    // LK7015 节点号(反馈 0x141)

/* 顺时针2: 右前 */
#define CHASSIS_STEER_FR_CAN_ID 0x52u
#define CHASSIS_STEER_FR_MST_ID 0x02u
#define CHASSIS_DRIVE_FR_NODE   2u // 反馈 0x142

/* 顺时针3: 右后 */
#define CHASSIS_STEER_RR_CAN_ID 0x53u
#define CHASSIS_STEER_RR_MST_ID 0x03u
#define CHASSIS_DRIVE_RR_NODE   3u // 反馈 0x143

/* 顺时针4: 左后 */
#define CHASSIS_STEER_RL_CAN_ID 0x54u
#define CHASSIS_STEER_RL_MST_ID 0x04u
#define CHASSIS_DRIVE_RL_NODE   4u // 反馈 0x144

/* 编译期校验: 约定 MST_ID 与 CAN_ID 使用同一编号(低4位一致), 改ID时漏改一侧会在这里直接报错 */
#if (((CHASSIS_STEER_FL_MST_ID ^ CHASSIS_STEER_FL_CAN_ID) | \
      (CHASSIS_STEER_FR_MST_ID ^ CHASSIS_STEER_FR_CAN_ID) | \
      (CHASSIS_STEER_RL_MST_ID ^ CHASSIS_STEER_RL_CAN_ID) | \
      (CHASSIS_STEER_RR_MST_ID ^ CHASSIS_STEER_RR_CAN_ID)) & 0x0fu)
#error "CHASSIS_STEER_x_MST_ID 的低4位必须等于对应 CHASSIS_STEER_x_CAN_ID 的低4位"
#endif

/* 底盘速度、跟随控制和舵电机 MIT 参数 */
#define SW_VXY       5.00f   // 遥控平移上限，m/s
#define SW_WZ        5.00f   // 跟随角速度上限，rad/s
#define SW_FOLLOW_KP 0.03f   // (rad/s)/deg
#define SW_ROTATE_WZ 1.50f   // rad/s
#define SW_VWHEEL    0.60f   // 轮缘速度上限，m/s
#define SW_PMAX      12.5f   // MIT 参数 PMAX，电机侧 rad（达妙调试助手里配的输出轴量程）
#define SW_VMAX      30.0f   // VMAX，rad/s
#define SW_TMAX      10.0f   // TMAX，N·m
#define SW_KP        3.0f
#define SW_KD        0.35f

/* 舵电机输出轴 → 舵轴(转向立柱)的换算:
 * 电机反馈 ±SW_PMAX(=±12.5) 对应舵轴 ±180°(±π)，即舵传动比 12.5/π ≈ 3.979。
 * 底盘内部（解算、零位、限幅、校准判断）统一使用舵轴角度 rad，范围 ±SW_STEER_MAX_RAD；
 * 只有进/出 DMMotorSendPosition 时换算回电机侧 rad，所以 PMAX 仍须与电机配置一致。
 * 注意: 该换算只处理比例，方向仍由 CHASSIS_x_STEER_DIR 决定（当前四轮都是 +1）。 */
#define SW_STEER_MAX_RAD 3.14159265f                                   // 舵轴角度上限，rad
#define SW_STEER_RAD_PER_MOTOR_RAD (SW_STEER_MAX_RAD / SW_PMAX)        // 电机侧 rad → 舵轴 rad
#define SW_MOTOR_RAD_PER_STEER_RAD (SW_PMAX / SW_STEER_MAX_RAD)        // 舵轴 rad → 电机侧 rad

/* 底盘几何尺寸（mm）和传动比 */
#define WHEEL_BASE 375              // 纵向轴距(前进后退方向)mm
#define TRACK_WIDTH 375             // 横向轮距(左右平移方向)mm
#define CENTER_GIMBAL_OFFSET_X 0    // 云台中心前后偏移（预留）mm
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台中心左右偏移（预留）mm
#define RADIUS_WHEEL 52.5           // 轮子半径mm
#define REDUCTION_RATIO_WHEEL 19.0f // 轮电机减速比（当前轮毂驱动未使用）

/* 各轮朝前时的舵轴角度（rad）, 顺序同轮位: 顺时针 FL/FR/RR/RL
 * 单位是舵轴 rad(±π)；用校准功能把前进姿态存成电机零点后，这里保持 0 即可。 */
#define CHASSIS_FL_ZERO_RAD 0.0f  // 左前
#define CHASSIS_FR_ZERO_RAD 0.0f  // 右前
#define CHASSIS_RR_ZERO_RAD 0.0f  // 右后
#define CHASSIS_RL_ZERO_RAD 0.0f  // 左后

/* ==================== 各轮转向 / 驱动极性 ====================
 * 只取 ±1。单轮装反(或电机正方向与车体约定相反)时改这里, 不用动解算代码。
 * 舵电机: +1 表示电机输出轴正转方向 = 轮组逆时针(俯视), 与 steering.c 的
 *         atan2(y, x) 约定一致; 装反的轮改成 -1, 解算会自动取镜像角。
 * 驱动电机: +1 表示 speed_dps 为正时轮子把车往前推; 装反改成 -1。
 * 注意: 这里只表示方向(±1)。电机输出轴到舵轴的减速比由上面的 SW_STEER_* 换算宏
 * 统一处理, 不要写成 -2.0f 之类的倍数, 否则会被下面的编译期检查拦下。 */

/* 台架实测: 四个舵电机顺时针(俯视)转时 position_rad 都增加, 即"电机正转 = 轮组顺时针",
 * 与 atan2 的逆时针为正相反, 所以四轮都取 -1。 */
#define CHASSIS_FL_STEER_DIR (-1) // 左前
#define CHASSIS_FR_STEER_DIR (-1) // 右前
#define CHASSIS_RR_STEER_DIR (-1) // 右后
#define CHASSIS_RL_STEER_DIR (-1) // 左后

/* 台架实测: 舵角全 0 时把车向前推, 1、4 号轮 speed_dps 为正, 2、3 号为负
 * (左右轮毂镜像安装), 所以右侧两个轮取 -1。 */
#define CHASSIS_FL_DRIVE_DIR (1)  // 左前: 正转为前进
#define CHASSIS_FR_DRIVE_DIR (-1) // 右前: 反转为前进
#define CHASSIS_RR_DRIVE_DIR (-1) // 右后: 反转为前进
#define CHASSIS_RL_DRIVE_DIR (1)  // 左后: 正转为前进

/* 极性只允许 ±1, 写成 0/2 之类会直接编译报错 */
#if ((CHASSIS_FL_STEER_DIR != 1) && (CHASSIS_FL_STEER_DIR != -1)) || \
    ((CHASSIS_FR_STEER_DIR != 1) && (CHASSIS_FR_STEER_DIR != -1)) || \
    ((CHASSIS_RR_STEER_DIR != 1) && (CHASSIS_RR_STEER_DIR != -1)) || \
    ((CHASSIS_RL_STEER_DIR != 1) && (CHASSIS_RL_STEER_DIR != -1)) || \
    ((CHASSIS_FL_DRIVE_DIR != 1) && (CHASSIS_FL_DRIVE_DIR != -1)) || \
    ((CHASSIS_FR_DRIVE_DIR != 1) && (CHASSIS_FR_DRIVE_DIR != -1)) || \
    ((CHASSIS_RR_DRIVE_DIR != 1) && (CHASSIS_RR_DRIVE_DIR != -1)) || \
    ((CHASSIS_RL_DRIVE_DIR != 1) && (CHASSIS_RL_DRIVE_DIR != -1))
#error "CHASSIS_x_STEER_DIR / CHASSIS_x_DRIVE_DIR 只能取 1 或 -1"
#endif

#define CHASSIS_STEER_HOLD_MPS   0.01f  // 轮速低于此值时保持上次舵角
#define CHASSIS_STEER_MARGIN_RAD 0.05f  // 舵电机位置量程两端的预留量

/* 底盘控制时序（ms） */
#define CHASSIS_CONTROL_PERIOD_MS   5u
#define CHASSIS_FEEDBACK_TIMEOUT_MS 60u   // 上位机指令(200Hz)的有效期

/* 舵电机时序。达妙 MIT 反馈是"一发一收"：收到一帧控制/失能帧才回一帧，
 * 所以空闲时的反馈周期由失能帧重发间隔决定，反馈新鲜度门限必须明显大于它，
 * 否则(60ms 周期 vs 60ms 门限)线上掩码会在 0/1 之间抖动，校准永远凑不齐四轮。 */
#define CHASSIS_STEER_IDLE_PERIOD_MS     20u  // 失能帧重发间隔，同时也是空闲时的反馈周期
#define CHASSIS_STEER_FEEDBACK_TIMEOUT_MS 120u // 舵电机反馈新鲜度门限(6 个反馈周期)

/* 无力档随舵电机探测周期发送 0x280 零力矩，便于观察 LK 反馈。置 0 可在无力档关闭轮毂。 */
#define CHASSIS_DRIVE_IDLE_KEEPALIVE 1
#define CHASSIS_DRIVE_ENABLE_REPEAT_MS 500u // 使能帧重发间隔：电机可能比 MCU 晚上电，需周期性重发

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
