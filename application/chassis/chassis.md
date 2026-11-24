# 四舵轮底盘

车体系 x 向前、y 向左，逆时针为正。消息 `chassis_cmd` 中的平移速度单位为 m/s，角速度为 rad/s，`offset_angle` 为度。单板通过消息中心传输，双板通过 CANComm 传输。两端须使用相同版本的 `Chassis_Upload_Data_s`。

| 调试器下标 | 轮位 | CAN2 舵向 MIT TX / RX | CAN1 轮毂节点 / RX |
| --- | --- | --- | --- |
| WHEEL_FL | 左前 | 0x001 / 0x011 | 1 / 0x141 |
| WHEEL_FR | 右前 | 0x002 / 0x012 | 2 / 0x142 |
| WHEEL_RL | 左后 | 0x003 / 0x013 | 3 / 0x143 |
| WHEEL_RR | 右后 | 0x004 / 0x014 | 4 / 0x144 |

`ChassisInit` 注册四个轮组。可从 `wheels[i].steer_motor` 查看 MIT 测量值和目标，从 `wheels[i].drive_motor` 查看轮毂测量值和目标速度。各轮机械零位和 MIT 量程在 [chassis.h](chassis.h) 中配置，量程须与实际驱动参数相符。安装方向和有符号传动比在 `wheel_config` 中配置。

`ChassisTask` 读取指令、处理底盘模式，并提交整条车体速度命令。`ChassisMotorTask` 在电机任务中每隔至少 5 ms 获取一次指令和反馈快照，调用 `steering.c` 做运动学解算，再发送本周期的四个舵向目标和一帧轮速广播。LK 应启用广播模式。启停使用 0x288，速度使用 0x281，各节点占两个字节，小端有符号，单位为 °/s。

启动时以零增益探测舵电机、零速探测轮毂。收到全部有效反馈后，以各轮实测舵角作为初始保持角。故障进入 `SWERVE_FAULT`，收到停机命令后可重新启动。停机命令入队失败会重试。

`chassis_feed` 中的 `swerve_phase` 为底盘阶段。`steer_online_mask` 和 `drive_online_mask` 表示最近 60 ms 收到过相应反馈的轮位，`fault_wheel_mask` 记录触发停机的轮位。位序与上表一致。广播发送失败或整条命令过期影响四轮，对应掩码为 0x0F。

[达妙官方资料](https://gitee.com/kit-miao/damiao) · [瓴控 CAN 广播协议 V2.35](http://www.lkmotor.cn/upload/20260317100100f.pdf) · [瓴控 CAN 单电机协议 V2.36](http://www.lkmotor.cn/upload/20260326142653f.pdf)
