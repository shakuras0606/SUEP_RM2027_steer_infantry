#include "chassis.h"
#include "robot_def.h"
#include "steering.h"
#include "steer_calibration.h"
#include "buzzer.h"
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

typedef struct {
    uint16_t steer_tx, steer_rx;
    uint8_t drive_node;
    SwerveGeometry_s geometry;
} WheelConfig_s;

static const WheelConfig_s wheel_config[WHEEL_COUNT] = {
    /* CAN_ID / MST_ID / LK节点号 见 chassis.h, 轮心 x/y, 安装零位, 有符号舵传动比, 轮向 */
    [WHEEL_FL] = {CHASSIS_STEER_FL_CAN_ID, CHASSIS_STEER_FL_MST_ID, CHASSIS_DRIVE_FL_NODE,
                  { HALF_X_M,  HALF_Y_M, CHASSIS_FL_ZERO_RAD, 1, 1}},
    [WHEEL_FR] = {CHASSIS_STEER_FR_CAN_ID, CHASSIS_STEER_FR_MST_ID, CHASSIS_DRIVE_FR_NODE,
                  { HALF_X_M, -HALF_Y_M, CHASSIS_FR_ZERO_RAD, 1, 1}},
    [WHEEL_RL] = {CHASSIS_STEER_RL_CAN_ID, CHASSIS_STEER_RL_MST_ID, CHASSIS_DRIVE_RL_NODE,
                  {-HALF_X_M,  HALF_Y_M, CHASSIS_RL_ZERO_RAD, 1, 1}},
    [WHEEL_RR] = {CHASSIS_STEER_RR_CAN_ID, CHASSIS_STEER_RR_MST_ID, CHASSIS_DRIVE_RR_NODE,
                  {-HALF_X_M, -HALF_Y_M, CHASSIS_RR_ZERO_RAD, 1, 1}},
};

typedef struct {
    DMMotorInstance *steer_motor;
    LK7015MotorInstance *drive_motor;
} SwerveWheelInstance;

typedef struct {
    float vx, vy, wz;
    uint32_t updated_ms;
    uint8_t enabled, steer_zero_permitted, steer_zero_request;
} MotionCommand_s;

typedef struct {
    float position_rad[WHEEL_COUNT];
    uint32_t steer_updated_ms[WHEEL_COUNT];
    uint8_t steer_online, drive_online, steer_enabled, steer_disabled, steer_fault;
} WheelFeedback_s;

static SwerveWheelInstance wheels[WHEEL_COUNT];
static LK7015Group_s drive_group;
static Referee_Interactive_info_t ui;

/* 机器人任务提交整条命令，并读取电机任务上次发布的诊断快照。 */
static volatile MotionCommand_s command;
static volatile uint8_t stop_requested;
static volatile uint8_t steer_zero_revoked;
static volatile Chassis_Upload_Data_s published_feedback;

/* 下列状态（含上一有效舵角）均由电机任务持有。 */
static SwerveTarget_s targets[WHEEL_COUNT];
static SwervePhase_e phase;
static SteerCalibration_s calibration;
static uint8_t fault_mask, init_fault_mask, stop_pending = 1;
static uint32_t start_ms, last_stop_ms;

void ChassisInit(void)
{
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        const WheelConfig_s *cfg = &wheel_config[i];
        CAN_Init_Config_s can = {.can_handle = CHASSIS_STEER_CAN, .tx_id = cfg->steer_tx, .rx_id = cfg->steer_rx};
        DMMotorLimits_s limits = {SW_PMAX, SW_VMAX, SW_TMAX};
        wheels[i].steer_motor = DMMotorInit(can, limits);
        wheels[i].drive_motor = LK7015MotorInit(&drive_group, CHASSIS_DRIVE_CAN, cfg->drive_node);
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
    static uint32_t last_board_cmd_ms;
    if (CANCommIsOnline(board_link)) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (board_link->update_flag)
            last_board_cmd_ms = HAL_GetTick();
        cmd = *(Chassis_Ctrl_Cmd_s *)CANCommGet(board_link);
        __set_PRIMASK(mask);
        if (HAL_GetTick() - last_board_cmd_ms > CHASSIS_FEEDBACK_TIMEOUT_MS)
            cmd.steer_zero_permitted = 0;
    }
#else
    SubGetMessage(cmd_sub, &cmd);
#endif

    MotionCommand_s next = {.vx = cmd.vx, .vy = cmd.vy, .wz = cmd.wz, .enabled = 1};
    switch (cmd.chassis_mode) {
    case CHASSIS_ZERO_FORCE:
        next.enabled = 0;
        next.steer_zero_permitted = cmd.steer_zero_permitted;
        next.steer_zero_request = cmd.steer_zero_request;
        break;
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
    if (!next.steer_zero_permitted && command.steer_zero_permitted)
        steer_zero_revoked = 1;
    if (next.steer_zero_permitted && !next.steer_zero_request)
        next.steer_zero_request = command.steer_zero_request; // 松杆时保留电机任务尚未取走的请求
    if (!next.steer_zero_permitted)
        next.steer_zero_request = 0;
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
        result.steer_updated_ms[i] = dm->updated_ms;
        if (dm->valid && now - dm->updated_ms <= CHASSIS_FEEDBACK_TIMEOUT_MS) {
            result.steer_online |= bit;
            if (dm->state == 0) result.steer_disabled |= bit;
            if (dm->state == 1) result.steer_enabled |= bit;
            if (dm->state >= 8) result.steer_fault |= bit;
        }
        if (lk->valid && now - lk->updated_ms <= CHASSIS_FEEDBACK_TIMEOUT_MS)
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
    if (stop_pending || now - last_stop_ms >= CHASSIS_FEEDBACK_TIMEOUT_MS) {
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
    if (now - motion.updated_ms > CHASSIS_FEEDBACK_TIMEOUT_MS ||
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
        if (now - start_ms > CHASSIS_START_TIMEOUT_MS) {
            TripFault(missing ? missing : ALL_WHEELS, now);
            return;
        }
        if (missing || now - start_ms <= CHASSIS_FEEDBACK_TIMEOUT_MS) {
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

/* 等待停机帧入队期间，命令和电机反馈都可能更新。设零前重新取快照。 */
static void UpdateSteerCalibration(const WheelFeedback_s *f, const MotionCommand_s *motion, uint32_t now)
{
    SteerCalibrationPhase_e previous = calibration.phase;
    uint8_t disabled = phase == SWERVE_OFF && !stop_pending ? f->steer_disabled : 0;
    uint8_t permitted = motion->steer_zero_permitted && now - motion->updated_ms <= CHASSIS_FEEDBACK_TIMEOUT_MS;
    WheelIndex_e wheel = SteerCalibrationStep(&calibration, permitted, motion->steer_zero_request, disabled,
                                              f->position_rad, f->steer_updated_ms, now);
    if (wheel != WHEEL_COUNT) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        now = HAL_GetTick();
        uint8_t allowed = !steer_zero_revoked && command.steer_zero_permitted &&
                          now - command.updated_ms <= CHASSIS_FEEDBACK_TIMEOUT_MS &&
                          ReadWheels(now).steer_disabled == ALL_WHEELS;
        __set_PRIMASK(mask);
        uint8_t queued = allowed && DMMotorSaveZero(wheels[wheel].steer_motor);
        SteerCalibrationSent(&calibration, queued, HAL_GetTick());
    }
    if (calibration.phase != previous) {
        if (calibration.phase == STEER_ZERO_SUCCESS)
            BuzzerPlayResult(BUZZER_RESULT_SUCCESS);
        else if (calibration.phase == STEER_ZERO_FAILED)
            BuzzerPlayResult(BUZZER_RESULT_FAILURE);
    }
}

void ChassisMotorTask(void)
{
    static uint32_t last_cycle_ms;
    uint32_t now = HAL_GetTick();
    if (now - last_cycle_ms < CHASSIS_CONTROL_PERIOD_MS)
        return;
    last_cycle_ms = now;

    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    MotionCommand_s motion = command;
    if (steer_zero_revoked)
        motion.steer_zero_permitted = 0;
    steer_zero_revoked = 0;
    command.steer_zero_request = 0;
    uint8_t stop = stop_requested;
    stop_requested = 0;
    now = HAL_GetTick();
    WheelFeedback_s f = {0};
    if (!init_fault_mask)
        f = ReadWheels(now);
    __set_PRIMASK(mask);

    if (calibration.phase != STEER_ZERO_IDLE && calibration.phase != STEER_ZERO_SUCCESS)
        motion.enabled = 0; // 校准中或失败后保持停机，完整重校成功才允许启动
    if (init_fault_mask) {
        phase = SWERVE_FAULT;
        fault_mask = init_fault_mask;
    } else {
        UpdateWheels(motion, stop, &f, now);
    }

    UpdateSteerCalibration(&f, &motion, now);

    mask = __get_PRIMASK();
    __disable_irq();
    published_feedback.swerve_phase = phase;
    published_feedback.steer_online_mask = f.steer_online;
    published_feedback.drive_online_mask = f.drive_online;
    published_feedback.fault_wheel_mask = fault_mask;
    published_feedback.steer_zero_state = calibration.phase;
    published_feedback.steer_zero_confirmed_mask = calibration.confirmed_mask;
    published_feedback.steer_zero_failed_mask = calibration.failed_mask;
    __set_PRIMASK(mask);
}
