# shoot

发射应用。摩擦轮为 **C620 + 2×M3508**(DJI 智能电机),拨弹轮为 **达妙 DM4310(MIT 模式)**,
分别由 `dji_motor` 和 `dmmotor` 模块驱动。

## 硬件

| 机构 | 型号/模式 | CAN | ID | 驱动模块 |
| --- | --- | --- | --- | --- |
| 左摩擦轮 | M3508 + C620,速度闭环 | CAN2 | 1 | `dji_motor` |
| 右摩擦轮 | M3508 + C620,速度闭环 | CAN2 | 2 | `dji_motor` |
| 拨弹轮 | DM4310 / MIT 模式 | CAN2 | ESC_ID=5(控制帧0x305,反馈帧0x015) | `dmmotor` |

> DM4310 需在达妙调试助手中配置:MODE=MIT、ESC_ID=5、MST_ID=0x015、
> PMAX/VMAX/TMAX 与 `shoot.h` 中的 `LOADER_PMAX/VMAX/TMAX` 一致、1Mbps 波特率。

## 工作流程

初始化 3 个电机(摩擦轮 ×2、拨弹轮 ×1),订阅 `shoot_cmd` 话题(来自 robot_cmd)并发布 `shoot_feed` 话题。

1. 从 `shoot_cmd` 获取消息;
2. `SHOOT_OFF`:停止摩擦轮并失能拨弹轮;其余模式使能摩擦轮和拨弹轮(使能帧仅在模式切换时发送,避免总线拥塞);
3. 按 `load_mode` 控制拨弹轮:
   - `LOAD_STOP`:拨弹轮停在当前位置(MIT 位置保持,`kp=LOADER_POS_KP`);
   - `LOAD_1_BULLET`/`LOAD_3_BULLET`:**仅在模式切换的边沿**累加目标角度(1/3 发 × 36°),
     之后保持在目标位置,避免每周期重复逼近导致目标漂移;
   - `LOAD_BURSTFIRE`:MIT 速度模式,目标速度 = `shoot_rate × 36°/发`(rad/s),靠 `kd` 阻尼跟踪;
   - `LOAD_REVERSE`:MIT 速度模式反转退弹;
4. 按 `friction_mode` 和 `bullet_speed` 给摩擦轮设置速度参考(`SHOOT_FRICTION_SPEED_15/18/30`);
5. 图传/弹仓盖(预留),最后发布 `shoot_feed`。

## 参数整定要点

- **摩擦轮**:速度环输出直接作为 M3508 电流指令(-16384~16384 对应 ±20A),
  当前只有 `SPEED_LOOP`(`close_loop_type = SPEED_LOOP`),`speed_PID` 为占位初值,
  必须实测弹道后重新整定 `SHOOT_FRICTION_SPEED_*` 与 PID(目标是摩擦轮转速稳定、弹速达标)。
- **拨弹轮**:MIT 的 `LOADER_POS_KP/KD`、`LOADER_SPEED_KD` 为占位初值。
  单发模式靠电机内部位置环定位,若拨盘过冲/打齿,先减小 `LOADER_POS_KP` 或加大 `kd`;
  连发若拨盘转不动,加大 `LOADER_SPEED_KD` 或检查 `LOADER_TMAX/VMAX` 是否与调试助手一致。
- 单发/三发当前**没有冷却时间**,完全由 cmd 的 `load_mode` 边沿触发;
  若需要发射冷却/弹仓保护,应在此处或 `robot_cmd` 中增加状态机。
