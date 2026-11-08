#include "fan_control.h" /* 引入与硬件无关的控制状态和接口。 */

#define FAN_RAMP_PERIOD_MS     200U  /* 软启动和按键停机每步的最短间隔。 */
#define FAN_TRACK_PERIOD_MS    100U  /* 正常闭环每次更新PWM的最短间隔。 */
#define FAN_DUTY_STEP           20U  /* 每步最多改变2个百分点风扇占空比。 */
#define FAN_RPM_DEADBAND        30U  /* 追踪目标时允许的RPM误差初值。 */
#define FAN_STOP_DROP_RPM       10U  /* 减速继续所需的最小反馈下降量。 */
#define FAN_STOP_TIMEOUT_MS   1000U  /* 一步减速无有效下降时直接请求0%。 */
#define FAN_START_TIMEOUT_MS 12000U  /* 软启动无测速的最大等待时间。 */
#define FAN_TACH_TIMEOUT_MS   1000U  /* 正常运行测速失效的额外等待上限。 */
#define FAN_STAGE_TIMEOUT_MS  5000U  /* 单个中间档等待上限，到期改为追踪最终目标。 */
#define FAN_STABLE_BAND_RPM     50U  /* 反馈进入最终目标±50RPM才确认稳定。 */
#define FAN_MIN_ERROR_RPM      100U  /* 稳定状态的最小允许转速偏差。 */
#define FAN_SPEED_ERROR_MS    2000U  /* 持续超出稳定保护范围的判定时间。 */

/* 目标真正改变或恢复运行时，等待新测速重算方向和阶段。 */
static void Fan_Control_ResetTransition(Fan_Control_t *control)
{
    control->control_target_rpm = 0U;
    control->stage_threshold_rpm = 0U;
    control->transition_step_ms = 0U;
    control->speed_error_since_ms = 0U;
    control->speed_error_active = 0U;
    control->state = (control->duty_permille == 0U) ?
                     FAN_STATE_STARTING : FAN_STATE_SWITCHING;
}

/* 每次启动或取消减速都从当前输出继续，避免恢复时占空比跳变。 */
static void Fan_Control_Resume(Fan_Control_t *control, uint32_t now_ms)
{
    control->start_ms = now_ms;
    control->last_valid_ms = now_ms;
    control->step_ms = now_ms - FAN_RAMP_PERIOD_MS;
    control->faults = 0U;
    control->start_seeded = 0U;
    Fan_Control_ResetTransition(control); /* 按键重新启动后重新建立稳定资格。 */
}

/* 异常退出时请求0%并锁住旋钮；保留目标和异常，等待用户再次按键。 */
static void Fan_Control_Fail(Fan_Control_t *control, uint8_t fault)
{
    control->faults |= fault;
    control->duty_permille = 0U;
    control->control_target_rpm = 0U;
    control->speed_error_active = 0U;
    control->enabled = 0U;
    control->state = FAN_STATE_STOPPED;
}

/* 从当前风扇占空比下调一步，并建立下一次反馈比较的基准。 */
static void Fan_Control_StopStep(Fan_Control_t *control,
                                  const Fan_Feedback_t *feedback, uint32_t now_ms)
{
    control->duty_permille = (control->duty_permille > FAN_DUTY_STEP) ?
                             control->duty_permille - FAN_DUTY_STEP : 0U;
    control->step_ms = now_ms;
    control->stop_rpm = feedback->rpm;
    control->stop_sequence = feedback->sequence;
    control->stop_reference = feedback->valid;
    control->state = (control->duty_permille == 0U) ?
                     FAN_STATE_STOPPED : FAN_STATE_STOPPING;
}

/* 相邻阶段最多相差一档，后续只按这两个阶段的固定中点判定推进。 */
static void Fan_Control_SetNextStage(Fan_Control_t *control, uint16_t from_rpm,
                                     uint32_t now_ms)
{
    int32_t delta = (int32_t)control->target_rpm - (int32_t)from_rpm; /* 本阶段到最终目标的有符号差值。 */
    uint16_t next_rpm = control->target_rpm; /* 距离不足一档时直接追踪最终目标。 */

    if (delta > (int32_t)FAN_RPM_STEP)
    {
        next_rpm = (uint16_t)(from_rpm + FAN_RPM_STEP);
    }
    else if (delta < -(int32_t)FAN_RPM_STEP)
    {
        next_rpm = (uint16_t)(from_rpm - FAN_RPM_STEP);
    }
    control->control_target_rpm = next_rpm;
    control->stage_threshold_rpm = (uint16_t)((from_rpm + next_rpm) / 2U);
    control->transition_step_ms = now_ms;
}

/* 只消费新有效测速，先追上已经跨过的半程，再判断是否接近最终目标。 */
static void Fan_Control_TrackTransition(Fan_Control_t *control, uint32_t rpm,
                                        uint32_t now_ms)
{
    if (control->control_target_rpm == 0U)
    {
        uint16_t from_rpm = (rpm > FAN_RPM_MAX) ? FAN_RPM_MAX : (uint16_t)rpm; /* 用当前实测重算方向，并限制中间档范围。 */
        Fan_Control_SetNextStage(control, from_rpm, now_ms); /* 首段以新目标后的可信实测为起点。 */
    }
    while (control->control_target_rpm != control->target_rpm)
    {
        uint8_t rising = (control->control_target_rpm < control->target_rpm); /* 剩余阶段的推进方向。 */

        if (((rising != 0U) && (rpm < control->stage_threshold_rpm)) ||
            ((rising == 0U) && (rpm > control->stage_threshold_rpm)))
        {
            break;
        }
        Fan_Control_SetNextStage(control, control->control_target_rpm, now_ms); /* 实测已越过半程，以下发的中间档为下一段起点。 */
    }
    if ((control->control_target_rpm == control->target_rpm) &&
        (rpm + FAN_STABLE_BAND_RPM >= control->target_rpm) &&
        (rpm <= (uint32_t)control->target_rpm + FAN_STABLE_BAND_RPM))
    {
        control->state = FAN_STATE_STABLE;
    }
}

/* 只在稳定状态统计新有效样本的连续超差时间，回到范围内就重新计时。 */
static uint8_t Fan_Control_SpeedFaultDue(Fan_Control_t *control, uint32_t rpm,
                                         uint32_t now_ms)
{
    uint32_t limit = control->target_rpm / 10U; /* 允许范围取最终目标的10%与100RPM中较大者。 */

    if (limit < FAN_MIN_ERROR_RPM)
    {
        limit = FAN_MIN_ERROR_RPM;
    }
    if ((rpm + limit >= control->target_rpm) && (rpm <= (uint32_t)control->target_rpm + limit))
    {
        control->speed_error_active = 0U;
        return 0U;
    }
    if (control->speed_error_active == 0U)
    {
        control->speed_error_since_ms = now_ms;
        control->speed_error_active = 1U;
    }
    return (uint8_t)((uint32_t)(now_ms - control->speed_error_since_ms) >= FAN_SPEED_ERROR_MS);
}

/* 初始化用户目标和启动时间；Flash读取由user_Init中的存储模块负责。 */
void Fan_Control_Init(Fan_Control_t *control, uint16_t target_rpm, uint32_t now_ms)
{
    control->target_rpm = ((target_rpm <= FAN_RPM_MAX) &&
                           ((target_rpm % FAN_RPM_STEP) == 0U)) ?
                          target_rpm : FAN_RPM_DEFAULT;
    control->duty_permille = 0U;
    control->enabled = 1U;
    control->faults = 0U;
    control->rotation_ms = now_ms;
    control->rotation_serial = 0U;
    control->sample_sequence = 0U;
    control->start_sequence = 0U;
    control->stop_sequence = 0U;
    control->stop_rpm = 0U;
    control->stop_reference = 0U;
    /* 建立软启动基准；0RPM目标会在首次更新时维持0%请求。 */
    Fan_Control_Resume(control, now_ms);
}

/* 单个按键事件只翻转一次；同次采样的旋转不与按键合并应用。 */
void Fan_Control_Input(Fan_Control_t *control, int16_t steps,
                       uint8_t button_pressed, uint8_t rotated, uint32_t now_ms)
{
    if (button_pressed != 0U)
    {
        control->enabled = (control->enabled == 0U) ? 1U : 0U;
        if (control->enabled != 0U)
        {
            /* 取消减速或重新启动时恢复原目标，不重放暂停期间的旋转。 */
            Fan_Control_Resume(control, now_ms);
        }
        return;
    }
    if (control->enabled == 0U)
    {
        return;
    }
    if ((rotated != 0U) || (steps != 0))
    {
        control->rotation_ms = now_ms;
        control->rotation_serial++;
    }
    if (steps != 0)
    {
        int32_t target = (int32_t)control->target_rpm +
                          ((int32_t)steps * (int32_t)FAN_RPM_STEP); /* 合并本次整格步进后限幅。 */
        if (target < 0)
        {
            target = 0;
        }
        if (target > (int32_t)FAN_RPM_MAX)
        {
            target = FAN_RPM_MAX;
        }
        if ((uint16_t)target != control->target_rpm)
        {
            control->target_rpm = (uint16_t)target;
            if ((target != 0) && ((control->state == FAN_STATE_STOPPED) ||
                                  (control->state == FAN_STATE_STOPPING)))
            {
                Fan_Control_Resume(control, now_ms); /* 从0档升速，按当前输出恢复。 */
            }
            else if (target == 0)
            {
                /* 立即应用0档，即使同一控制轮又收到一个向上的旋钮事件也能正常重启。 */
                control->duty_permille = 0U;
                control->control_target_rpm = 0U;
                control->speed_error_active = 0U;
                control->state = FAN_STATE_STOPPED;
            }
            else
            {
                control->control_target_rpm = 0U;
                control->speed_error_active = 0U;
                if (control->state == FAN_STATE_STABLE)
                {
                    control->state = FAN_STATE_SWITCHING;
                }
            }
        }
    }
}

/* 控制器由固定短周期调用，等待通过状态与时间戳实现，不阻塞任务。 */
void Fan_Control_Update(Fan_Control_t *control, const Fan_Feedback_t *feedback,
                        uint32_t now_ms)
{
    if (control->target_rpm == 0U)
    {
        /* 旋钮0档直接请求风扇端0%，保留旋转使能。 */
        control->duty_permille = 0U;
        control->state = FAN_STATE_STOPPED;
        return;
    }
    if (control->enabled == 0U)
    {
        control->control_target_rpm = 0U;
        control->speed_error_active = 0U;
        if (control->duty_permille == 0U)
        {
            control->state = FAN_STATE_STOPPED;
            return;
        }
        if (control->state != FAN_STATE_STOPPING)
        {
            /* 按键停机先减小一步，再等待新的下降反馈。 */
            Fan_Control_StopStep(control, feedback, now_ms);
            return;
        }
        if ((uint32_t)(now_ms - control->step_ms) >= FAN_STOP_TIMEOUT_MS)
        {
            control->faults |= FAN_FAULT_STOP_TIMEOUT;
            if (feedback->valid == 0U)
            {
                control->faults |= FAN_FAULT_TACH_LOST;
            }
            control->duty_permille = 0U;
            control->state = FAN_STATE_STOPPED;
            return;
        }
        if (((uint32_t)(now_ms - control->step_ms) >= FAN_RAMP_PERIOD_MS) &&
            (control->stop_reference != 0U) && (feedback->valid != 0U) &&
            ((uint32_t)(feedback->sequence - control->stop_sequence) >= 2U) &&
            (control->stop_rpm >= FAN_STOP_DROP_RPM) &&
            (feedback->rpm <= (control->stop_rpm - FAN_STOP_DROP_RPM)))
        {
            /* 只接受本步骤之后的新测速，达到下降门槛才继续下调。 */
            Fan_Control_StopStep(control, feedback, now_ms);
        }
        return;
    }

    if (control->start_seeded == 0U)
    {
        control->start_sequence = feedback->sequence;
        control->start_seeded = 1U;
    }
    if ((control->start_seeded == 1U) && (feedback->valid != 0U) &&
        ((uint32_t)(feedback->sequence - control->start_sequence) >= 2U))
    {
        control->start_seeded = 2U;
    }
    if ((feedback->valid != 0U) && (control->start_seeded == 2U))
    {
        control->last_valid_ms = now_ms;
        if (control->state == FAN_STATE_STARTING)
        {
            control->state = FAN_STATE_SWITCHING;
        }
    }
    if (control->state == FAN_STATE_STARTING)
    {
        if ((uint32_t)(now_ms - control->start_ms) >= FAN_START_TIMEOUT_MS)
        {
            /* 始终没有启动反馈时不无限增大输出，锁定并请求0%。 */
            Fan_Control_Fail(control, FAN_FAULT_TACH_LOST);
            return;
        }
        if ((uint32_t)(now_ms - control->step_ms) >= FAN_RAMP_PERIOD_MS)
        {
            control->duty_permille = (control->duty_permille < (1000U - FAN_DUTY_STEP)) ?
                                     control->duty_permille + FAN_DUTY_STEP : 1000U;
            control->step_ms = now_ms;
        }
        return;
    }
    if ((feedback->valid == 0U) || (control->start_seeded != 2U))
    {
        control->speed_error_active = 0U;
        if ((uint32_t)(now_ms - control->last_valid_ms) >= FAN_TACH_TIMEOUT_MS)
        {
            /* 正常运行失去反馈后转为停止请求，避免盲目继续加速。 */
            Fan_Control_Fail(control, FAN_FAULT_TACH_LOST);
        }
        return;
    }
    if ((control->state == FAN_STATE_SWITCHING) && (control->control_target_rpm != 0U) &&
        (control->control_target_rpm != control->target_rpm) &&
        ((uint32_t)(now_ms - control->transition_step_ms) >= FAN_STAGE_TIMEOUT_MS))
    {
        /* 中间档最多等5秒。到期先追踪最终目标，仍用新的有效测速确认稳定和更新PWM。 */
        control->control_target_rpm = control->target_rpm;
    }
    if (feedback->sequence == control->sample_sequence)
    {
        return;
    }
    control->sample_sequence = feedback->sequence;
    if (control->state == FAN_STATE_SWITCHING)
    {
        Fan_Control_TrackTransition(control, feedback->rpm, now_ms); /* 半程推进独立于PWM写入节奏。 */
    }
    else if ((control->state == FAN_STATE_STABLE) &&
             (Fan_Control_SpeedFaultDue(control, feedback->rpm, now_ms) != 0U)) /* 仅稳定状态允许超差停机。 */
    {
        control->faults |= FAN_FAULT_SPEED_DEVIATION;
        control->enabled = 0U;
        control->control_target_rpm = 0U;
        control->speed_error_active = 0U;
        Fan_Control_StopStep(control, feedback, now_ms); /* 复用按键减速停机的首步和后续反馈保护。 */
        return;
    }

    if ((uint32_t)(now_ms - control->step_ms) < FAN_TRACK_PERIOD_MS)
    {
        return;
    }
    control->step_ms = now_ms;
    if ((feedback->rpm + FAN_RPM_DEADBAND) < control->control_target_rpm)
    {
        control->duty_permille = (control->duty_permille < (1000U - FAN_DUTY_STEP)) ?
                                 control->duty_permille + FAN_DUTY_STEP : 1000U;
    }
    else if (feedback->rpm > ((uint32_t)control->control_target_rpm + FAN_RPM_DEADBAND))
    {
        control->duty_permille = (control->duty_permille > FAN_DUTY_STEP) ?
                                 control->duty_permille - FAN_DUTY_STEP : 0U;
    }
}

/* 灯条显示目标值；按键停止时在减速期间也立即全灭。 */
uint8_t Fan_Control_LedCount(const Fan_Control_t *control)
{
    return (control->enabled != 0U) ? (uint8_t)(control->target_rpm / 300U) : 0U;
}

/* 保存由旋转事件驱动，静止无新事件时不会每3秒重复写Flash。 */
uint8_t Fan_Control_SaveDue(const Fan_Control_t *control, uint16_t saved_rpm,
                            uint8_t saved_valid, uint32_t *attempt_serial,
                            uint32_t now_ms)
{
    if ((control->rotation_serial == *attempt_serial) ||
        ((uint32_t)(now_ms - control->rotation_ms) < 3000U) ||
        ((saved_valid != 0U) && (control->target_rpm == saved_rpm)))
    {
        return 0U;
    }
    *attempt_serial = control->rotation_serial;
    return 1U;
}
