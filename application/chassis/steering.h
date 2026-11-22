#pragma once
#include <stdint.h>
#include "bsp_can.h"

/* LK7015 轮毂电机 */
#define LK_CAN_HANDLE (&hcan1)
#define LK_SPEED_TX  0x281u   /* 四电机速度广播 */
#define LK_MODE_TX   0x288u   /* 四电机启停广播 */
#define LK_FL_RX     0x141u
#define LK_FR_RX     0x142u
#define LK_RL_RX     0x143u
#define LK_RR_RX     0x144u

/* DM4310 舵向电机，MIT 模式。TX = CAN_ID，RX = MST_ID */
#define DM_CAN_HANDLE (&hcan2)
#define DM_FL_TX     0x001u//前左
#define DM_FR_TX     0x002u//前右
#define DM_RL_TX     0x003u//后左
#define DM_RR_TX     0x004u//后右
#define DM_FL_RX     0x011u//前左
#define DM_FR_RX     0x012u//前右
#define DM_RL_RX     0x013u//后左
#define DM_RR_RX     0x014u//后右

#define SW_VXY       0.40f   /* 摇杆满量程平移速度，m/s */
#define SW_WZ        1.00f   /* 摇杆满量程角速度，rad/s */
#define SW_VWHEEL    0.60f   /* 单轮轮缘速度上限，m/s */

#define SW_RATIO     1.0f    /* DM输出轴转角 / 舵轮方位角 */
#define SW_PMAX      12.5f   /* 必须与达妙上位机中的 PMAX 一致 */
#define SW_KP        3.0f    /* MIT 位置增益，架空起调值 */
#define SW_KD        0.35f   /* MIT 阻尼增益，架空起调值 */

void SwerveInit(void);
void SwerveControl(float vx, float vy, float wz, uint8_t enable);
