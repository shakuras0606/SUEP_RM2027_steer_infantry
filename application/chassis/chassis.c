/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 接收车体速度命令，驱动四个舵轮模组。x 向前，y 向左。
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "steering.h"
#include "message_center.h"
#include "referee_task.h"

#ifdef CHASSIS_BOARD
#include "can_comm.h"
static CANCommInstance *board_link;
#else
static Subscriber_t *cmd_sub;
static Publisher_t *feed_pub;
#endif

static Referee_Interactive_info_t ui;
static Chassis_Upload_Data_s feedback;

void ChassisInit(void)
{
    SwerveInit();
    UITaskInit(&huart1, &ui);

#ifdef CHASSIS_BOARD
    INS_Init();
    CANComm_Init_Config_s c = {
        .can_config = {.can_handle = &hcan2, .tx_id = 0x311, .rx_id = 0x312},
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s)
    };
    board_link = CANCommInit(&c);
#else
    cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    feed_pub = PubRegister("chassis_feed", sizeof(feedback));
#endif
}

void ChassisTask(void)
{
    Chassis_Ctrl_Cmd_s cmd = {0};

#ifdef CHASSIS_BOARD
    if (CANCommIsOnline(board_link))
        cmd = *(Chassis_Ctrl_Cmd_s *)CANCommGet(board_link);
#else
    SubGetMessage(cmd_sub, &cmd);
#endif

    SwerveControl(cmd.vx, cmd.vy, cmd.wz,
                  cmd.chassis_mode == CHASSIS_NO_FOLLOW);

#ifdef CHASSIS_BOARD
    CANCommSend(board_link, &feedback);
#else
    PubPushMessage(feed_pub, &feedback);
#endif
}
