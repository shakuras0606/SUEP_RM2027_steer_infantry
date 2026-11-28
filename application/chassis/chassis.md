# 四舵轮底盘

车体系 x 向前、y 向左，逆时针为正。消息 `chassis_cmd` 中的平移速度单位为 m/s，角速度为 rad/s，`offset_angle` 为度。单板通过消息中心传输，双板通过 CANComm 传输。两端须使用相同版本的 `Chassis_Upload_Data_s`。

轮位下标固定为顺时针顺序：左前(1) → 右前(2) → 右后(3) → 左后(4)。该顺序同时是舵电机 CAN_ID、反馈帧和轮毂节点号的编号，三者必须一致，改 ID 时在 [chassis.h](chassis.h) 里同步修改。

| 调试器下标 | 轮位 | 舵电机 motors[] | CAN2 舵向 TX / RX | CAN1 轮毂节点 / RX |
| --- | --- | --- | --- | --- |
| WHEEL_FL | 左前（顺时针1） | motors[1] | 0x051 / 0x001 | 1 / 0x141 |
| WHEEL_FR | 右前（顺时针2） | motors[2] | 0x052 / 0x002 | 2 / 0x142 |
| WHEEL_RR | 右后（顺时针3） | motors[3] | 0x053 / 0x003 | 3 / 0x143 |
| WHEEL_RL | 左后（顺时针4） | motors[4] | 0x054 / 0x004 | 4 / 0x144 |

`motors[0]` 是拨弹轮（见 shoot 模块），不在本表内；`motors[5..7]` 是 DM 驱动的预留空槽，不会更新。

`ChassisInit` 注册四个轮组。可从 `wheels[i].steer_motor` 查看 MIT 测量值和目标，从 `wheels[i].drive_motor` 查看轮毂测量值和目标速度。各轮机械零位和 MIT 量程在 [chassis.h](chassis.h) 中配置，量程须与实际驱动参数相符。

### 舵角换算、零位与校准

舵电机回传的位置是**电机侧 rad**，量程为 MIT 参数 `SW_PMAX`（当前 12.5）。机械上该量程正好对应舵轴 ±180°，所以 [chassis.h](chassis.h) 用 `SW_STEER_RAD_PER_MOTOR_RAD = π / 12.5 ≈ 3.979` 把电机侧换算成**舵轴 rad（±π）**：

- 反馈进入底盘时换算成舵轴角度（`ReadWheels`），之后解算、限幅、零位、校准判断全部使用舵轴 rad；
- 目标下发前再乘 `SW_MOTOR_RAD_PER_STEER_RAD` 换回电机侧 rad（`UpdateSteer`）；
- 因此 `CHASSIS_x_ZERO_RAD` 的单位也是舵轴 rad，`SW_PMAX` 必须继续等于电机实际配置的量程（它同时是 MIT 帧的限幅依据）。

零点不需要写死在 `*_ZERO_RAD` 里：把四个轮子摆成朝前姿态，进入零力模式（遥控器左摇杆推到最前并保持 1 s）即触发校准，逐个电机发送 `0xFE` 存零，之后电机在该姿态回传 0，底盘即以该姿态为 0°。校准只在四轮都确认失能后才能存零，失败时 `steer_zero_failed_mask` 会标出出问题的轮位（bit0~bit3 = FL/FR/RR/RL）。

### 转向 / 驱动极性

某个轮子转反了，只改 [chassis.h](chassis.h) 里对应的一个 `±1` 宏，不要动 `steering.c` 的解算：

| 宏 | 作用 |
| --- | --- |
| `CHASSIS_FL_STEER_DIR` … `CHASSIS_RL_STEER_DIR` | 舵电机输出轴正方向与轮组逆时针（俯视）方向是否同向，取 -1 表示装反 |
| `CHASSIS_FL_DRIVE_DIR` … `CHASSIS_RL_DRIVE_DIR` | 轮毂转速为正时是否把车往前推，取 -1 表示装反 |

这四个 `*_STEER_DIR` 会写进 `SwerveGeometry_s.steer_ratio`（含方向的传动比），`*_DRIVE_DIR` 写进 `drive_direction`；解算里 `angle = (position_rad - zero_rad) / steer_ratio`、`speed_mps = drive_direction * …`，所以改符号等效于把该轮的舵角和轮速一起镜像。极性写成 0 或 2 会在编译期被 `#error` 拦下。电机输出轴到舵轴之间的减速比不写在这里（`steer_ratio` 保持 ±1），而是由 `SW_STEER_RAD_PER_MOTOR_RAD` / `SW_MOTOR_RAD_PER_STEER_RAD` 统一处理。

本车实测的极性（已经写进 [chassis.h](chassis.h)）：

- **舵向四轮全为 -1**：电机正转时轮组是顺时针（俯视），而解算约定逆时针为正，故取反。
- **驱动为 +1 / -1 / -1 / +1**：轮毂左右镜像安装，舵角全 0 时把车向前推，1、4 号轮 `speed_dps` 为正、2、3 号为负。

验证方法（车架空）：只给 `vx > 0` 时四轮应保持 0° 不转；只给 `vy > 0`（向左平移）时四轮应一起指向车身左侧。若指向右侧，说明舵向符号取反了，四个 `*_STEER_DIR` 一起改回 +1。

`ChassisTask` 读取指令、处理底盘模式，并提交整条车体速度命令。`ChassisMotorTask` 在电机任务中每隔至少 5 ms 获取一次指令和反馈快照，调用 `steering.c` 做运动学解算，再发送本周期的四个舵向目标和一帧力矩广播。LK 应启用广播模式。启停使用 0x288；该批驱动只支持广播力矩控制，所以轮速走 0x280——四个轮子各两个字节、小端有符号，是速度环 PID 的输出（协议范围 ±2000），测速取自 0x141~0x144 的回复。

遥控进入行驶档即进入 `SWERVE_RUNNING`。舵电机按各自的反馈上线，刚上线时以实测角为初始保持角。各轮毂独立检查测速反馈，超时只把自己的力矩清零。切回 `CHASSIS_ZERO_FORCE` 会失能所有舵电机并广播零力矩。校准只在无力档进行，失败结果可在反馈中查看。

架空单测轮毂时，选左开关下、右开关中的平移档。同轮舵角不可用时，右摇杆前后（`vx`）直接给该轮毂设定速度，保留本轮驱动极性。横移和旋转指令不参与这个单测给定。舵角恢复后，该轮自动回到舵轮解算。

`chassis_feed` 中的 `swerve_phase` 为遥控选择的底盘阶段。`steer_online_mask` 和 `drive_online_mask` 表示最近收到过反馈的轮位（舵向 120 ms、轮毂 60 ms）。`fault_wheel_mask` 记录最近一个输出周期的初始化、舵电机报错、解算或发送异常，仅用于诊断。bit0~bit3 依次为 FL / FR / RR / RL。驱动广播发送失败时掩码为 0x0F。

[达妙官方资料](https://gitee.com/kit-miao/damiao) · [瓴控 CAN 广播协议 V2.35](http://www.lkmotor.cn/upload/20260317100100f.pdf) · [瓴控 CAN 单电机协议 V2.36](http://www.lkmotor.cn/upload/20260326142653f.pdf)
