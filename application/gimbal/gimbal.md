# gimbal

云台应用。yaw/pitch 各使用一个**达妙 DM4310(一控四固件 V41)**,由 `dm4310_ctrl4` 模块驱动:
电机内部只有电流环,角度环/速度环在本应用配置的 PID 中实现,反馈来自 BMI088 IMU。

## 硬件

| 电机 | 型号/模式 | CAN | 电机ID | 反馈 |
| --- | --- | --- | --- | --- |
| yaw | DM4310 / 一控四(V41) | CAN3 | 1 | IMU `YawTotalAngle`、`Gyro[2]` |
| pitch | DM4310 / 一控四(V41) | CAN3 | 2 | IMU `Pitch`、`Gyro[0]` |

> 一控四电机需在达妙调试助手中预先配置电机ID(1/2)和 1Mbps 波特率。
> 电机ID 1~4 共用 `0x3FE` 控制帧,因此 yaw/pitch 必须是 1~4 号,不能使用 5~8 号(会跑到 `0x4FE` 组)。

## 工作流程

初始化两个 DM4310 电机与 IMU,订阅 `gimbal_cmd` 话题(来自 robot_cmd),发布 `gimbal_feed` 话题。

1. 从消息中心获取 `gimbal_cmd` 消息;
2. 根据 `gimbal_mode` 切换模式:急停则停止两个电机(下发 0 电流),其余模式使能并切换反馈来源为 `OTHER_FEED`(IMU);
3. 把 cmd 下发的 yaw/pitch 目标角(度)设置为角度环参考值;
4. 发布反馈数据:IMU 姿态 + yaw 电机单圈编码器值(供底盘跟随解算 `offset_angle`)。

## 参数整定要点

- **速度环输出量纲变了**:一控四的 PID 输出是标幺电流(满量程 ±10000 = ±Imax),而原 GM6020 是力矩值 ±30000。
  本工程速度环 `Kp/Ki/MaxOut` 已按 6020 参数等比例折算(如 yaw 50/200/20000 → 25/100/10000),
  **更换云台机械结构后必须重新整定**。
- 角度环输出是速度参考(度/秒),`MaxOut=500` 表示限速 500deg/s。
- 若云台方向与 IMU 正方向相反,先改 `GYRO2GIMBAL_DIR_*`,再考虑 `motor_reverse_flag`。
- 一控四固件设置过零点后绝对编码器值会改变,`gimbal.h` 中的 `YAW_CHASSIS_ALIGN_ECD`/`PITCH_HORIZON_ECD`
  必须重新测量,否则底盘跟随的 `offset_angle` 与云台零点都会偏移。
