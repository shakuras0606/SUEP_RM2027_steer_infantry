#include "chassis.h"
#include "robot_def.h"
#include "steering.h"
#include "dmmotor.h"
#include "lk7015_motor.h"
#include "message_center.h"
#include "referee_task.h"
#include <math.h>

#ifdef CHASSIS_BOARD
#include "can_comm.h"
static CANCommInstance *board_link;
#else
static Subscriber_t *cmd_sub;
static Publisher_t *feed_pub;
#endif

#define HALF_X_M (WHEEL_BASE * 0.0005f)
#define HALF_Y_M (TRACK_WIDTH * 0.0005f)
#define WHEEL_DPS_PER_MPS (57.29577951f / (RADIUS_WHEEL * 0.001f))
#define ALL_WHEELS ((1u << WHEEL_COUNT) - 1u)
#define CONTROL_PERIOD_MS 5u
#define FEEDBACK_TIMEOUT_MS 60u
#define START_TIMEOUT_MS 500u

typedef struct {
    uint16_t steer_tx, steer_rx;
    uint8_t drive_node;
    SwerveGeometry_s geometry;
} WheelConfig_s;

static const WheelConfig_s wheel_config[WHEEL_COUNT] = {
    /* MIT CAN_ID / MST_ID，LK 节点号，轮心 x/y，安装零位，有符号舵传动比，轮向 */
    [WHEEL_FL] = {0x001, 0x011, 1, { HALF_X_M,  HALF_Y_M, 0, 1, 1}},
    [WHEEL_FR] = {0x002, 0x012, 2, { HALF_X_M, -HALF_Y_M, 0, 1, 1}},
    [WHEEL_RL] = {0x003, 0x013, 3, {-HALF_X_M,  HALF_Y_M, 0, 1, 1}},
    [WHEEL_RR] = {0x004, 0x014, 4, {-HALF_X_M, -HALF_Y_M, 0, 1, 1}},
};

typedef struct {
    DMMotorInstance *steer_motor;
    LK7015MotorInstance *drive_motor;
} SwerveWheelInstance;

typedef struct {
    float vx, vy, wz;
    uint32_t updated_ms;
    uint8_t enabled;
} MotionCommand_s;

typedef struct {
    float position_rad[WHEEL_COUNT];
    uint8_t steer_online, drive_online, steer_enabled, steer_fault;
} WheelFeedback_s;

static SwerveWheelInstance wheels[WHEEL_COUNT];
static LK7015Group_s drive_group;
static Referee_Interactive_info_t ui;

/* 机器人任务提交整条命令，并读取电机任务上次发布的诊断快照。 */
static volatile MotionCommand_s command;
static volatile uint8_t stop_requested;
static volatile Chassis_Upload_Data_s published_feedback;

/* 下列状态（含上一有效舵角）均由电机任务持有。 */
static SwerveTarget_s targets[WHEEL_COUNT];
static SwervePhase_e phase;
static uint8_t fault_mask, init_fault_mask, stop_pending = 1;
static uint32_t start_ms, last_stop_ms;

void ChassisInit(void)
{
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        const WheelConfig_s *cfg = &wheel_config[i];
        CAN_Init_Config_s can = {.can_handle = &hcan2, .tx_id = cfg->steer_tx, .rx_id = cfg->steer_rx};
        DMMotorLimits_s limits = {SW_PMAX, SW_VMAX, SW_TMAX};
        wheels[i].steer_motor = DMMotorInit(can, limits);
        wheels[i].drive_motor = LK7015MotorInit(&drive_group, &hcan1, cfg->drive_node);
        if (!wheels[i].steer_motor || !wheels[i].drive_motor)
            init_fault_mask |= 1u << i;
    }
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
    feed_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif
}

void ChassisTask(void)
{
    Chassis_Ctrl_Cmd_s cmd = {0};
#ifdef CHASSIS_BOARD
    if (CANCommIsOnline(board_link)) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        cmd = *(Chassis_Ctrl_Cmd_s *)CANCommGet(board_link);
        __set_PRIMASK(mask);
    }
#else
    SubGetMessage(cmd_sub, &cmd);
#endif

    MotionCommand_s next = {.vx = cmd.vx, .vy = cmd.vy, .wz = cmd.wz, .enabled = 1};
    switch (cmd.chassis_mode) {
    case CHASSIS_FOLLOW_GIMBAL_YAW:
        next.wz = -SW_FOLLOW_KP * cmd.offset_angle;
        if (next.wz > SW_WZ) next.wz = SW_WZ;
        if (next.wz < -SW_WZ) next.wz = -SW_WZ;
        break;
    case CHASSIS_ROTATE:
        next.wz = SW_ROTATE_WZ;
        break;
    case CHASSIS_NO_FOLLOW:
        break;
    default:
        next.enabled = 0;
        break;
    }
    next.updated_ms = HAL_GetTick();
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    command = next;
    if (!next.enabled)
        stop_requested = 1;   /* 即使下一条命令先到，也要执行这次停机 */
    Chassis_Upload_Data_s feedback = published_feedback;
    __set_PRIMASK(mask);

#ifdef CHASSIS_BOARD
    CANCommSend(board_link, &feedback);
#else
    PubPushMessage(feed_pub, &feedback);
#endif
}

/* 调用者短暂屏蔽 CAN 中断，取得同一时刻的各轮测量快照。 */
static WheelFeedback_s ReadWheels(uint32_t now)
{
    WheelFeedback_s result = {0};
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        const volatile DMMotorMeasure_s *dm = &wheels[i].steer_motor->measure;
        const volatile LK7015Measure_s *lk = &wheels[i].drive_motor->measure;
        uint8_t bit = 1u << i;
        result.position_rad[i] = dm->position_rad;
        if (dm->valid && now - dm->updated_ms <= FEEDBACK_TIMEOUT_MS) {
            result.steer_online |= bit;
            if (dm->state == 1) result.steer_enabled |= bit;
            if (dm->state >= 8) result.steer_fault |= bit;
        }
        if (lk->valid && now - lk->updated_ms <= FEEDBACK_TIMEOUT_MS)
            result.drive_online |= bit;
    }
    return result;
}

static uint8_t SetWheelMode(uint8_t enabled)
{
    uint8_t failed = LK7015SetEnabled(&drive_group, enabled) ? 0 : ALL_WHEELS;
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        if (!DMMotorSetEnabled(wheels[i].steer_motor, enabled))
            failed |= 1u << i;
    }
    return failed;
}

static void StopWheels(uint32_t now)
{
    if (stop_pending || now - last_stop_ms >= FEEDBACK_TIMEOUT_MS) {
        uint8_t failed = SetWheelMode(0);
        last_stop_ms = now;
        stop_pending = failed != 0;
        fault_mask |= failed;
        if (failed)
            phase = SWERVE_FAULT;
    }
}

static void TripFault(uint8_t failed, uint32_t now)
{
    phase = SWERVE_FAULT;
    fault_mask |= failed;
    stop_pending = 1;
    StopWheels(now);
}

static uint8_t SendTargets(uint8_t running)
{
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        DMMotorPositionRef_s ref = {0};
        float speed_dps = 0;
        if (running) {
            ref = (DMMotorPositionRef_s){targets[i].steer_rad, SW_KP, SW_KD};
            speed_dps = targets[i].speed_mps * WHEEL_DPS_PER_MPS;
        }
        if (!DMMotorSendPosition(wheels[i].steer_motor, ref))
            return 1u << i;
        LK7015MotorSetSpeed(wheels[i].drive_motor, speed_dps);
    }
    return LK7015SendSpeeds(&drive_group) ? 0 : ALL_WHEELS;
}

static void UpdateWheels(MotionCommand_s motion, uint8_t stop, const WheelFeedback_s *f, uint32_t now)
{
    if (!motion.enabled || stop) {
        if (phase != SWERVE_OFF) stop_pending = 1;
        phase = SWERVE_OFF;
        fault_mask = 0;
        StopWheels(now);
        return;
    }
    if (phase == SWERVE_FAULT) {
        StopWheels(now);
        return;
    }
    if (now - motion.updated_ms > FEEDBACK_TIMEOUT_MS ||
        !isfinite(motion.vx) || !isfinite(motion.vy) || !isfinite(motion.wz)) {
        TripFault(ALL_WHEELS, now);
        return;
    }
    if (f->steer_fault) {
        TripFault(f->steer_fault, now);
        return;
    }
    if (phase == SWERVE_OFF) {
        uint8_t failed = SetWheelMode(1);
        if (failed) {
            TripFault(failed, now);
            return;
        }
        start_ms = now;
        phase = SWERVE_STARTING;
        return;
    }

    uint8_t missing = ALL_WHEELS & ~(f->steer_enabled & f->drive_online);
    if (phase == SWERVE_STARTING) {
        if (now - start_ms > START_TIMEOUT_MS) {
            TripFault(missing ? missing : ALL_WHEELS, now);
            return;
        }
        if (missing || now - start_ms <= FEEDBACK_TIMEOUT_MS) {
            uint8_t failed = SendTargets(0);
            if (failed) TripFault(failed, now);
            return;
        }
        for (unsigned i = 0; i < WHEEL_COUNT; ++i)
            targets[i].steer_rad = f->position_rad[i];
        phase = SWERVE_RUNNING;
    }
    if (missing) {
        TripFault(missing, now);
        return;
    }
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        if (!SwerveSolveWheel(&wheel_config[i].geometry, motion.vx, motion.vy, motion.wz,
                              f->position_rad[i], SW_PMAX, &targets[i])) {
            TripFault(1u << i, now);
            return;
        }
    }
    SwerveLimitSpeeds(targets, SW_VWHEEL);
    uint8_t failed = SendTargets(1);
    if (failed) TripFault(failed, now);
}

void ChassisMotorTask(void)
{
    static uint32_t last_cycle_ms;
    uint32_t now = HAL_GetTick();
    if (now - last_cycle_ms < CONTROL_PERIOD_MS)
        return;
    last_cycle_ms = now;

    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    MotionCommand_s motion = command;
    uint8_t stop = stop_requested;
    stop_requested = 0;
    now = HAL_GetTick();
    WheelFeedback_s f = {0};
    if (!init_fault_mask)
        f = ReadWheels(now);
    __set_PRIMASK(mask);

    if (init_fault_mask) {
        phase = SWERVE_FAULT;
        fault_mask = init_fault_mask;
    } else {
        UpdateWheels(motion, stop, &f, now);
    }

    mask = __get_PRIMASK();
    __disable_irq();
    published_feedback.swerve_phase = phase;
    published_feedback.steer_online_mask = f.steer_online;
    published_feedback.drive_online_mask = f.drive_online;
    published_feedback.fault_wheel_mask = fault_mask;
    __set_PRIMASK(mask);
}
