#ifndef SHOOT_H
#define SHOOT_H

/* 电机接线（DJI 节点号） */
#define SHOOT_MOTOR_CAN         (&hcan2)
#define SHOOT_FRICTION_L_CAN_ID 1u
#define SHOOT_FRICTION_R_CAN_ID 2u
#define SHOOT_LOADER_CAN_ID     3u

/* 机构参数 */
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比,英雄需要修改为3508的19.0f
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