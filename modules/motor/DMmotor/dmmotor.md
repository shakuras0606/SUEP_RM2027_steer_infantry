# dmmotor

达妙(DM)电机 **MIT 控制模式** 驱动模块。负责 MIT 报文的收发、量程换算和 MIT 内部增益(KP/KD)的发送,
不包含软件 PID —— 位置/速度/力矩的闭环由电机固件内部完成,主控只下发目标值。

支持同一 CAN 总线上挂多个 DM 电机(各自独立控制帧/反馈帧),本工程用于:

- 底盘 4 个转向 DM4310(`application/chassis/chassis.c`,控制帧 `0x051~0x054`,反馈帧 `0x001~0x004`,顺时针 FL/FR/RR/RL)
- 拨弹轮 DM4310(`application/shoot/shoot.c`,控制帧 `0x305`,反馈帧 `0x015`)

---

## 1. 硬件与电机配置要求

需先在**达妙调试助手**中配置(本工程不写电机参数):

| 项目 | 要求 |
| --- | --- |
| 控制模式 | CTRL_MODE = 1 (MIT),模式切换会清除当前指令 |
| ESC_ID | 电机侧控制帧按 `0x300 + ESC_ID` 应答;本工程发送的控制帧见下方注 |
| MST_ID | 反馈帧 ID,须满足 **MST_ID 低 4 位 = ESC_ID 低 4 位**(驱动用它校验反馈帧) |
| PMAX/VMAX/TMAX | 必须与初始化时传入的 `DMMotorLimits_s` 完全一致,否则位置/速度/力矩整体比例错误 |
| CAN 波特率 | 1Mbps,与主控一致 |
| 超时保护 | 建议配置 TIMEOUT,主控掉线后电机可自动卸力 |

> **底盘转向电机的控制帧是 `0x051~0x054`,不是 `0x300+ESC_ID`。** 这两者可混用是因为 V13 系列固件的
> CAN 过滤器只对低 8 位匹配:`0x351`(= `0x300+0x51`)的低 8 位就是 `0x51`,与 `0x051` 等价。
> 所以 `chassis.h` 里 `CHASSIS_STEER_*_CAN_ID` 填的是 `0x51~0x54`(ESC_ID 本身),驱动直接用它在总线上发帧,
> 电机仍按 `0x351~0x354` 受理。**反过来,若电机 ESC_ID 只填 `0x01`,则控制帧 `0x051` 匹配不到**,必须改成 `0x51`。

### 1.1 MIT 报文格式

MIT 控制帧(`0x300+ESC_ID`,8 字节),数据区为 5 个字段按位拼接:

```
D[0..1] = p_des (16bit) | D[2..3]高4位 = v_des(12bit) ...
实际布局: p_des(16) | v_des(12) | kp(12) | kd(12) | t_ff(12)
```

- `p_des`、`v_des`、`t_ff` 按 `[-PMAX,+PMAX]`、`[-VMAX,+VMAX]`、`[-TMAX,+TMAX]` 线性映射到无符号定点;
- `kp` 范围 `0~500`,`kd` 范围 `0~5`,超出会被驱动拒绝并 `LOGERROR`;
- 反馈帧为 `MST_ID`,8 字节:`D[0]` 低 4 位为电机 ID(高 4 位为错误码),`D[1..2]` 位置,`D[3..4]` 速度,
  `D[4..5]` 力矩,`D[6]` MOS 温度,`D[7]` 转子温度。

### 1.2 使能/失能

使能帧为 `0xFF×7 + 0xFC`,失能帧为 `0xFF×7 + 0xFD`(标准 DM 协议)。
电机失能后为自由拖拽状态;`DMMotorSetEnabled()` 内部会记录状态并清零参考值。

## 2. 类型定义

```c
typedef struct
{
    float position_rad, velocity_rad_s, torque_nm; // 量程,必须与电机内 PMAX/VMAX/TMAX 一致
} DMMotorLimits_s;

typedef struct
{
    float position_rad, velocity_rad_s, torque_nm;
    uint8_t state, mos_temp_c, rotor_temp_c;
    uint32_t updated_ms;
    uint8_t valid;
} DMMotorMeasure_s;

typedef struct
{
    float position_rad, kp, kd;
} DMMotorPositionRef_s;

typedef struct
{
    volatile DMMotorMeasure_s measure;
    DMMotorPositionRef_s reference;
    DMMotorLimits_s limits;
    CANInstance *motor_can_instance;
    uint8_t enable_requested;
} DMMotorInstance;
```

## 3. 外部接口

```c
DMMotorInstance *DMMotorInit(CAN_Init_Config_s can, DMMotorLimits_s limits);
uint8_t DMMotorSetEnabled(DMMotorInstance *motor, uint8_t enabled); // 1=使能 0=失能
uint8_t DMMotorSendPosition(DMMotorInstance *motor, DMMotorPositionRef_s reference);
uint8_t DMMotorSendVelocity(DMMotorInstance *motor, float velocity_rad_s, float kd);
```

- `DMMotorInit()` 的 `can.tx_id` 填**控制帧 ID**(如 `0x305`),`can.rx_id` 填**反馈帧 ID**(如 `0x015`);
  `limits` 必须与电机内配置一致。
- `DMMotorSendPosition()`:发送 MIT 位置目标(`v_des=0`,`t_ff=0`),`kp/kd` 为电机内部位置刚度/阻尼。
  适合拨弹轮单发定位、底盘转向保持等场合。
- `DMMotorSendVelocity()`:发送 MIT 速度目标(`kp=0`),仅靠 `kd` 阻尼跟踪 `v_des`,
  适合拨弹轮连发/反转等需要“匀速转动”而非“定位”的场合。
  内部用当前反馈位置作为 `p_des`(未收到反馈时按 0 位置),因此不会因位置目标漂移而撞限位。
- 三个发送接口都有安全检查:参数非有限值、超量程、未初始化指针时拒绝发送并 `LOGERROR`,返回 0;
  正常发送返回 1。
- `DM_MOTOR_COUNT` 为本模块最多允许的实例数,新增超过该数量时初始化失败,需要同步调大。

## 4. 使用示例

### 4.1 位置控制(单发拨弹)

```c
CAN_Init_Config_s can = {.can_handle = &hcan2, .tx_id = 0x305, .rx_id = 0x015};
DMMotorLimits_s limits = {.position_rad = 12.5f, .velocity_rad_s = 30.0f, .torque_nm = 10.0f};
DMMotorInstance *loader = DMMotorInit(can, limits);

DMMotorSetEnabled(loader, 1);                                   // 使能
float target = loader->measure.position_rad + 36.0f * DEGREE_2_RAD; // 一次拨弹的角度
DMMotorSendPosition(loader, (DMMotorPositionRef_s){.position_rad = target, .kp = 8.0f, .kd = 0.5f});
```

### 4.2 速度控制(连发)

```c
DMMotorSendVelocity(loader, 28.0f, 0.3f); // 28rad/s 正转,KD=0.3
DMMotorSendVelocity(loader, -5.0f, 0.3f); // 反转退弹
```

> 注意:MIT 速度跟踪依赖电机的 `kd` 阻尼项,`kd` 太小时速度跟不住、负载变化时波动大;
> 但 `kd` 过大会把电机“捏”得很硬,连发时可能出现堵转/弹丸挤压,需实际调试。

## 5. 调试建议

1. 先 `DMMotorSetEnabled(m,1)` 下发 `kp=0,kd=0.2` 的零力矩位置,确认电机不飞车、反馈 `state` 正常;
2. 逐步加大 `kp` 测试定位,观察 `measure.position_rad` 是否稳定在目标附近;
3. `measure.velocity_rad_s`、`torque_nm` 可用于判断是否堵转/过载;
4. 若电机完全没反应:检查 `MST_ID` 与 `ESC_ID` 低 4 位是否一致(`DecodeDMMotor()` 会校验 `data[0]&0x0F == tx_id&0x0F`)、
   控制帧 ID 是否为 `0x300+ESC_ID`、波特率是否为 1Mbps。
