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
    /* 按下标顺序(顺时针 FL/FR/RR/RL)排列, CAN_ID / MST_ID / LK节点号 / 极性 见 chassis.h
     * 其余字段: 轮心 x/y, 安装零位, 有符号舵传动比, 轮向 */
    [WHEEL_FL] = {CHASSIS_STEER_FL_CAN_ID, CHASSIS_STEER_FL_MST_ID, CHASSIS_DRIVE_FL_NODE,
                  { HALF_X_M,  HALF_Y_M, CHASSIS_FL_ZERO_RAD, CHASSIS_FL_STEER_DIR, CHASSIS_FL_DRIVE_DIR}},
    [WHEEL_FR] = {CHASSIS_STEER_FR_CAN_ID, CHASSIS_STEER_FR_MST_ID, CHASSIS_DRIVE_FR_NODE,
                  { HALF_X_M, -HALF_Y_M, CHASSIS_FR_ZERO_RAD, CHASSIS_FR_STEER_DIR, CHASSIS_FR_DRIVE_DIR}},
    [WHEEL_RR] = {CHASSIS_STEER_RR_CAN_ID, CHASSIS_STEER_RR_MST_ID, CHASSIS_DRIVE_RR_NODE,
                  {-HALF_X_M, -HALF_Y_M, CHASSIS_RR_ZERO_RAD, CHASSIS_RR_STEER_DIR, CHASSIS_RR_DRIVE_DIR}},
    [WHEEL_RL] = {CHASSIS_STEER_RL_CAN_ID, CHASSIS_STEER_RL_MST_ID, CHASSIS_DRIVE_RL_NODE,
                  {-HALF_X_M,  HALF_Y_M, CHASSIS_RL_ZERO_RAD, CHASSIS_RL_STEER_DIR, CHASSIS_RL_DRIVE_DIR}},
};

typedef struct {
    DMMotorInstance *steer_motor;
    LK7015MotorInstance *drive_motor;
} SwerveWheelInstance;

typedef struct {
    float vx, vy, wz, bench_speed_mps;
    uint32_t updated_ms;
    uint8_t enabled, steer_zero_permitted, steer_zero_request;
} MotionCommand_s;

typedef struct {
    float position_rad[WHEEL_COUNT];
    uint32_t steer_updated_ms[WHEEL_COUNT];
    uint8_t steer_online, drive_online, steer_disabled, steer_fault;
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
static uint8_t fault_mask, init_fault_mask, steer_ready, stop_pending = 1;

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
    static Chassis_Ctrl_Cmd_s cmd;
    static uint32_t received_ms;
#ifdef CHASSIS_BOARD
    if (CANCommIsOnline(board_link)) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (board_link->update_flag)
            received_ms = HAL_GetTick();
        cmd = *(Chassis_Ctrl_Cmd_s *)CANCommGet(board_link);
        __set_PRIMASK(mask);
    } else {
        cmd = (Chassis_Ctrl_Cmd_s){.chassis_mode = CHASSIS_ZERO_FORCE};
    }
#else
    if (SubGetMessage(cmd_sub, &cmd))
        received_ms = HAL_GetTick();
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
        next.bench_speed_mps = cmd.vx;
        break;
    default:
        next.enabled = 0;
        break;
    }
    next.updated_ms = received_ms;
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
        uint8_t bit = 1u << i;
        const DMMotorInstance *steer = wheels[i].steer_motor;
        const LK7015MotorInstance *drive = wheels[i].drive_motor;
        if (steer && steer->measure.valid && now - steer->measure.updated_ms <= CHASSIS_STEER_FEEDBACK_TIMEOUT_MS) {
            result.steer_online |= bit;
            result.position_rad[i] = steer->measure.position_rad * SW_STEER_RAD_PER_MOTOR_RAD;
            result.steer_updated_ms[i] = steer->measure.updated_ms;
            if (!steer->enable_requested) result.steer_disabled |= bit;
            if (steer->measure.state >= 8) result.steer_fault |= bit;
        }
        if (drive && drive->measure.valid && now - drive->measure.updated_ms <= LK7015_FEEDBACK_TIMEOUT_MS)
            result.drive_online |= bit;
    }
    return result;
}

static void UpdateSteer(unsigned i, const MotionCommand_s *motion, const WheelFeedback_s *f, uint8_t probe)
{
    uint8_t bit = 1u << i;
    const SwerveGeometry_s *geometry = &wheel_config[i].geometry;
    DMMotorInstance *steer = wheels[i].steer_motor;
    DMMotorPositionRef_s ref = {0};

    /* 架空单测时，缺少舵角的轮毂直接响应前后摇杆。 */
    targets[i].speed_mps = motion->bench_speed_mps * geometry->drive_direction;
    if (!steer)
        return;
    if (!(f->steer_online & bit) || (f->steer_fault & bit)) {
        steer_ready &= ~bit;
        if (!probe)
            return;
        if (!(f->steer_fault & bit) && !DMMotorSetEnabled(steer, 1))
            fault_mask |= bit;
        /* 零增益探测，使晚接入的舵电机能回传当前位置。 */
    } else {
        if (!(steer_ready & bit)) {
            targets[i].steer_rad = f->position_rad[i];
            if (!DMMotorSetEnabled(steer, 1)) {
                fault_mask |= bit;
                return;
            }
            steer_ready |= bit;
        }
        if (SwerveSolveWheel(geometry, motion->vx, motion->vy, motion->wz,
                             f->position_rad[i], SW_STEER_MAX_RAD, &targets[i])) {
            ref = (DMMotorPositionRef_s){targets[i].steer_rad * SW_MOTOR_RAD_PER_STEER_RAD, SW_KP, SW_KD};
        } else {
            fault_mask |= bit;
        }
    }
    if (!DMMotorSendPosition(steer, ref))
        fault_mask |= bit;
}

static void UpdateWheels(MotionCommand_s motion, uint8_t stop, const WheelFeedback_s *f, uint32_t now)
{
    static uint32_t last_probe_ms, last_drive_mode_ms;
    uint8_t running = motion.enabled && !stop;
    SwervePhase_e next = running ? SWERVE_RUNNING : SWERVE_OFF;
    uint8_t changed = next != phase;
    uint8_t immediate = changed || stop_pending;
    uint8_t probe = immediate || now - last_probe_ms >= CHASSIS_STEER_IDLE_PERIOD_MS;
    phase = next;
    if (!running && !probe)
        return;
    if (probe)
        last_probe_ms = now;

    stop_pending = 0;
    fault_mask = init_fault_mask | f->steer_fault;
    if (drive_group.sender.can_handle &&
        (immediate || now - last_drive_mode_ms >= CHASSIS_DRIVE_ENABLE_REPEAT_MS)) {
        if (LK7015SetEnabled(&drive_group, running || CHASSIS_DRIVE_IDLE_KEEPALIVE)) {
            last_drive_mode_ms = now;
        } else {
            fault_mask |= ALL_WHEELS;
            if (!running) stop_pending = 1;
        }
    }

    if (running) {
        /* 过期或无效的速度给定回中，电机仍保持当前遥控模式。 */
        if (now - motion.updated_ms > CHASSIS_FEEDBACK_TIMEOUT_MS ||
            !isfinite(motion.vx) || !isfinite(motion.vy) || !isfinite(motion.wz))
            motion.vx = motion.vy = motion.wz = motion.bench_speed_mps = 0.0f;
        for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
            UpdateSteer(i, &motion, f, probe);
            if (!(f->drive_online & (1u << i)))
                targets[i].speed_mps = 0.0f;
        }
        SwerveLimitSpeeds(targets, SW_VWHEEL);
    } else {
        steer_ready = 0;
        for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
            targets[i].speed_mps = 0.0f;
            if (wheels[i].steer_motor && !DMMotorSetEnabled(wheels[i].steer_motor, 0)) {
                fault_mask |= 1u << i;
                stop_pending = 1;
            }
        }
    }
    for (unsigned i = 0; i < WHEEL_COUNT; ++i) {
        if (wheels[i].drive_motor)
            LK7015MotorSetSpeed(wheels[i].drive_motor, targets[i].speed_mps * WHEEL_DPS_PER_MPS);
    }
    if (drive_group.sender.can_handle && (running || CHASSIS_DRIVE_IDLE_KEEPALIVE || immediate) &&
        !LK7015SpeedLoopStep(&drive_group, running)) {
        fault_mask |= ALL_WHEELS;
        if (!running) stop_pending = 1;
    }
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
    WheelFeedback_s f = ReadWheels(now);
    __set_PRIMASK(mask);

    UpdateWheels(motion, stop, &f, now);
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
