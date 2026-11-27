# dm4310_ctrl4

达妙 DM4310 **一控四固件(V41)** 的驱动模块。一控四固件的电机内部只有电流环,且同一组(最多4个)电机共用一帧电流控制报文;
本模块在软件层实现与 `dji_motor` 相同的 **角度环 → 速度环 → 标幺电流** 串级控制,因此可以像 GM6020 一样直接设置角度/速度参考值。

> 适用固件: DM4310 一控四固件(硬件版本 V3 `50xx`,固件版本 V41,例如 5041)。
> 普通 MIT/力位混控固件请使用同目录的 `dmmotor` 模块。

---

## 1. 硬件与电机配置要求

一控四模式下电机不再使用 MIT 报文,需先在**达妙调试助手**中完成配置(本工程不写电机参数):

| 项目 | 要求 |
| --- | --- |
| 工作模式 | 一控四固件上电自动进入电流环模式,无需设置 CTRL_MODE |
| 电机 ID | 1~8,同一 CAN 总线不允许重复;本驱动用 `can_init_config.tx_id` 传入电机 ID |
| CAN 波特率 | 默认 1Mbps,**必须与主控 FDCAN 一致**(`Core/Src/fdcan.c` 已配为 1Mbps) |
| PMAX/VMAX/TMAX | 一控四模式不使用,但电流满量程 = 电机 `Imax` |

### 1.1 控制报文(主控 → 电机)

| 电机 ID | 控制帧 CAN ID | 数据区 |
| --- | --- | --- |
| 1~4 | **0x3FE** | `D[2k]`=电流低8位, `D[2k+1]`=电流高8位, k = ID-1 |
| 5~8 | **0x4FE** | `D[2k]`=电流低8位, `D[2k+1]`=电流高8位, k = ID-5 |

- 电流为**标幺值**:`10000` 对应 `1.0 × Imax`,本驱动把 PID 输出限制在 `±10000`。
- > ⚠️ 官方 V41 文档只写了“控制电流为标幺值,放大10000倍,范围0-10000”(抄自力位混控的 `i_des` 说明,那里是**限幅值**),
  > 未明确正负号。本驱动按**有符号 int16 补码**发送(低字节在前),即 `-10000~+10000` 双向电流。
  > **首次调试必须架空电机、从小电流双向试探**:若负向指令无响应或反向飞车,说明该固件把负值当成了无符号大电流,
  > 需要改为“符号位单独处理/只发正值由方向标志翻转”的方案(修改 `DM4310Ctrl4Control()` 的打包处)。
- 注意字节序为**低字节在前**(与 DJI 电机相反),驱动内部已处理。
- 一控四固件没有独立的“使能/失能”报文,`DM4310Ctrl4Stop()` 的实现是**持续下发 0 电流**(电机自由拖拽),不会锁轴。

### 1.2 反馈报文(电机 → 主控)

反馈帧 CAN ID = **0x300 + 电机ID**,1ms 一帧,8 字节:

| 字节 | 含义 |
| --- | --- |
| D[0]<<8 \| D[1] | 位置编码器,0~8191(一圈),**高字节在前** |
| D[2]<<8 \| D[3] | 转速,有符号,单位 rpm×100 |
| D[4]<<8 \| D[5] | 转矩电流,有符号,单位 mA |
| D[6] | 线圈温度 ℃ |
| D[7] | 错误状态 |

驱动把位置换算为角度制、速度换算为 **度/秒**(`rpm×100×0.06`),与 `dji_motor` 的单位保持一致。

### 1.3 零点设置(可选)

一控四固件支持设置机械零点:向 `0x7FF` 发送标准帧 `D[0..3] = {CANID_L, CANID_H, 0x55, 0x50}`。
**设置零点会改变绝对编码器值**,因此云台 `YAW_CHASSIS_ALIGN_ECD` / `PITCH_HORIZON_ECD` 等标定值必须重新测量,
否则云台零点会整体偏移。

---

## 2. 类型定义

```c
typedef struct
{
    uint16_t last_ecd, ecd;     // 上一帧/当前帧编码器值(0~8191)
    float angle_single_round;   // 单圈角度,单位 度 (0~360)
    float speed_aps;            // 转速,单位 度/秒 (已低通)
    float current_mA;           // 转矩电流,单位 mA (已低通)
    uint8_t temperature;        // 线圈温度 ℃
    uint8_t error_state;        // 电机错误状态字节
    float total_angle;          // 多圈角度,单位 度
    int32_t total_round;        // 圈数计数
    uint32_t updated_ms;        // 最近一次收到反馈的系统时间(ms)
    uint8_t valid;              // 是否收到过反馈
} DM4310Ctrl4_Measure_s;

typedef struct
{
    volatile DM4310Ctrl4_Measure_s measure;
    Motor_Control_Setting_s motor_settings;   // 复用框架通用控制配置
    Motor_Controller_s motor_controller;      // 复用框架三环控制器(含PID和参考值)
    CANInstance *motor_can_instance;          // 本电机反馈帧对应的 CAN 实例
    CANInstance *tx_sender;                   // 所属控制组的发送实例(组内共享)
    uint8_t motor_id;                         // 1~8
    uint8_t sender_group;                     // 0: 0x3FE组, 1: 0x4FE组
    uint8_t message_num;                      // 组内槽位0~3
    Motor_Working_Type_e stop_flag;           // MOTOR_STOP 时下发0电流
    DaemonInstance *daemon;
    uint32_t feed_cnt;
    float dt;
} DM4310Ctrl4Instance;
```

## 3. 外部接口

```c
DM4310Ctrl4Instance *DM4310Ctrl4Init(Motor_Init_Config_s *config);   // 初始化,返回实例指针
void DM4310Ctrl4SetRef(DM4310Ctrl4Instance *motor, float ref);       // 设置最外层闭环参考值
void DM4310Ctrl4ChangeFeed(DM4310Ctrl4Instance *motor, Closeloop_Type_e loop, Feedback_Source_e type);
void DM4310Ctrl4OuterLoop(DM4310Ctrl4Instance *motor, Closeloop_Type_e outer_loop); // 修改外层闭环
void DM4310Ctrl4Stop(DM4310Ctrl4Instance *motor);                    // 停止:持续下发0电流
void DM4310Ctrl4Enable(DM4310Ctrl4Instance *motor);
void DM4310Ctrl4Control(void);                                       // 控制计算与CAN发送,周期调用
```

- `DM4310Ctrl4Init()` 的 `config->can_init_config.tx_id` 直接填**电机 ID(1~8)**,不是 CAN 报文 ID;
  驱动会自动把发送实例的 `tx_id` 改为对应组的控制帧(`0x3FE`/`0x4FE`)、`rx_id` 改为 `0x300+ID`。
- 同一控制组的第一个注册的电机充当该组的发送实例,其余电机共享它的 `tx_buff`;
  `DM4310Ctrl4Control()` 每个控制周期对每个控制组只发送一帧,同一帧最多携带 4 个电机的电流。
- **必须**在 `motor_task.c` 的 `MotorControlTask()` 里周期调用 `DM4310Ctrl4Control()`(本工程为 1kHz)。

## 4. 初始化与安全检查

初始化时会检查:

1. `config`/`can_handle` 非空;
2. 电机 ID 在 1~8;
3. 实例数量未超过 `DM4310_CTRL4_MOTOR_CNT`;
4. 使用 `OTHER_FEED` 时对应的反馈指针已赋值,启用前馈时前馈指针已赋值;
5. `outer_loop_type` 必须包含在 `close_loop_type` 中,否则参考值会被直接当作电流下发;
6. 同一总线上不允许出现相同电机 ID(否则会抢同一反馈帧)。

任一项失败会 `LOGERROR` 并返回 `NULL`,应用层应及时处理(见下方示例)。

## 5. 使用示例(本项目云台)

```c
Motor_Init_Config_s yaw_config = {
    .can_init_config = {
        .can_handle = GIMBAL_MOTOR_CAN,
        .tx_id = GIMBAL_YAW_MOTOR_ID,   // 达妙工具中配置的电机ID
    },
    .controller_param_init_config = {
        .angle_PID = {.Kp = 8, .Ki = 0, .Kd = 0, .DeadBand = 0.1,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 100, .MaxOut = 500},
        .speed_PID = {.Kp = 25, .Ki = 100, .Kd = 0,
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                      .IntegralLimit = 1500, .MaxOut = 10000}, // 输出为标幺电流
        .other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle,
        .other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2],
    },
    .controller_setting_init_config = {
        .angle_feedback_source = OTHER_FEED,
        .speed_feedback_source = OTHER_FEED,
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    },
};

DM4310Ctrl4Instance *motor = DM4310Ctrl4Init(&yaw_config);
if (motor == NULL)
    LOGERROR("gimbal yaw motor init failed");

// 控制周期内:
DM4310Ctrl4SetRef(motor, 90.0f);   // 角度环目标 90 度
```

> `PID` 输出的上限应按一控四的**标幺电流满量程 10000** 来设置,不能沿用 GM6020 的 20000/30000 量级,
> 否则输出会长期饱和。更换电机/负载后需重新整定。

## 6. 调试建议

1. 先让电机**架空**,只开 `SPEED_LOOP` 给一个小速度参考,确认转向与符号(`motor_reverse_flag`);
2. 确认速度反馈方向正确后,再开角度环,从小 `Kp` 开始加;
3. 检查 `measure.error_state`、`temperature` 是否异常,以及 `valid` 是否在 20ms 内持续刷新;
4. 一控四组内 4 个电机若有一个离线,该组控制报文仍会照常发送,其余电机不受影响。
