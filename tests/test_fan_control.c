/* 与硬件无关的风扇状态机测试；不加入目标固件。 */
#include "fan_control.h" /* 使用实机相同的控制逻辑。 */
#include <assert.h> /* 检查边界和异常处理。 */
#include <stdio.h> /* 输出主机测试汇总。 */

/** 建立已经取得有效新反馈、达到最终目标的稳定场景。 */
static void Test_Running(Fan_Control_t *control, uint16_t target,
                         uint16_t duty, uint32_t now_ms, uint32_t sequence)
{
    Fan_Control_Init(control, target, now_ms); /* 建立完整合法初值。 */
    control->state = FAN_STATE_STABLE;
    control->control_target_rpm = target;
    control->transition_step_ms = now_ms;
    control->stage_threshold_rpm = 0U;
    control->duty_permille = duty;
    control->step_ms = now_ms;
    control->last_valid_ms = now_ms;
    control->sample_sequence = sequence;
    control->start_sequence = sequence - 2U;
    control->start_seeded = 2U;
}

/** 检查默认、非法目标、0/1500边界及每一个整数RPM的LED阈值。 */
static void Test_InitAndLeds(void)
{
    Fan_Control_t control; /* 当前待测控制状态。 */
    Fan_Feedback_t no_feedback = {0U, 0U, 0U}; /* 没有测速时的硬件快照。 */
    uint16_t target; /* 遍历LED边界所需的目标值。 */
    Fan_Control_Init(&control, 600U, 0U); /* 空白Flash由上层传入默认600。 */
    assert(control.target_rpm == 600U && control.enabled == 1U); /* 开机自动使能。 */
    assert(control.duty_permille == 0U && control.state == FAN_STATE_STARTING); /* 从0输出进入软启动。 */
    Fan_Control_Init(&control, 650U, 0U); /* 输入非法非100倍数目标。 */
    assert(control.target_rpm == 600U); /* 非法档位回退默认。 */
    Fan_Control_Init(&control, 1501U, 0U); /* 输入超过上限目标。 */
    assert(control.target_rpm == 600U); /* 上限外目标回退默认。 */
    Fan_Control_Init(&control, 0U, 0U); /* 0是有效设定，不应回退默认。 */
    Fan_Control_Update(&control, &no_feedback, 0U); /* 0档保持0%请求。 */
    assert(control.target_rpm == 0U && control.enabled == 1U); /* 0档仍允许调节。 */
    assert(control.duty_permille == 0U && control.state == FAN_STATE_STOPPED); /* 输出0与使能状态分开。 */
    Fan_Control_Init(&control, 1500U, 0U); /* 上限是有效目标。 */
    assert(control.target_rpm == 1500U); /* 不因最高档回退。 */
    for (target = 0U; target <= 1500U; ++target)
    {
        control.target_rpm = target;
        assert(Fan_Control_LedCount(&control) == target / 300U); /* 覆盖299/300、599/600等所有边界。 */
    }
    control.target_rpm = 1500U;
    control.enabled = 0U;
    assert(Fan_Control_LedCount(&control) == 0U); /* 暂停立即全灭，不等待实际转速下降。 */
}

/** 检查暂停锁定、目标保留、按钮恢复、0档可调及步进限幅。 */
static void Test_InputsAndPause(void)
{
    Fan_Control_t control; /* 当前待测控制状态。 */
    Fan_Feedback_t feedback = {900U, 100U, 1U}; /* 暂停前有效测速。 */
    uint32_t serial; /* 暂停前的旋转事件版本。 */
    Test_Running(&control, 900U, 600U, 0U, 100U); /* 建立正常运行。 */
    serial = control.rotation_serial;
    Fan_Control_Input(&control, 3, 1U, 1U, 10U); /* 按键与旋转同次出现时优先处理按键。 */
    assert(control.enabled == 0U && control.target_rpm == 900U); /* 暂停保留原目标。 */
    assert(Fan_Control_LedCount(&control) == 0U); /* 按下时灯即灭。 */
    Fan_Control_Input(&control, -8, 0U, 1U, 15U); /* 模拟暂停期间继续转动旋钮。 */
    assert(control.target_rpm == 900U && control.rotation_serial == serial); /* 暂停旋转不改目标且不触发保存。 */
    Fan_Control_Update(&control, &feedback, 20U); /* 暂停后开始非阻塞减速。 */
    assert(control.duty_permille == 580U && control.state == FAN_STATE_STOPPING); /* 首次只减少2个百分点。 */
    Fan_Control_Input(&control, 0, 1U, 0U, 30U); /* 再按一次取消暂停，恢复保留目标。 */
    assert(control.enabled == 1U && control.target_rpm == 900U); /* 不恢复暂停期间旋转。 */
    assert(control.duty_permille == 580U); /* 重新启动不突然跳占空比。 */
    Fan_Control_Input(&control, 32767, 0U, 1U, 40U); /* 极大正向步进应限幅。 */
    assert(control.target_rpm == 1500U); /* 上限固定1500。 */
    Fan_Control_Input(&control, -32768, 0U, 1U, 50U); /* 极大负向步进应限幅且不整数溢出。 */
    assert(control.target_rpm == 0U && control.enabled == 1U); /* 到0档不锁住旋钮。 */
    Fan_Control_Update(&control, &feedback, 50U); /* 转到0档时直接下发0%请求。 */
    assert(control.duty_permille == 0U && control.state == FAN_STATE_STOPPED); /* 不进入按键停止的逐步减速。 */
    assert(control.enabled == 1U && control.faults == 0U); /* 0档保持旋钮使能，也不是故障。 */
    Fan_Control_Input(&control, 1, 0U, 1U, 60U); /* 从运行中选到的0档再向上转一格。 */
    assert(control.target_rpm == 100U && control.state == FAN_STATE_STARTING); /* 可以立即重新调速。 */
    Fan_Control_Init(&control, 0U, 100U); /* 独立检查上电0档。 */
    feedback.valid = 0U;
    Fan_Control_Update(&control, &feedback, 100U); /* 应用0档输出。 */
    Fan_Control_Input(&control, 1, 0U, 1U, 110U); /* 从0向上旋转一格。 */
    assert(control.target_rpm == 100U && control.enabled == 1U); /* 正常进入100档。 */
    assert(control.state == FAN_STATE_STARTING); /* 从0输出重新软启动。 */
}

/** 保存只在旋转结束满3秒后尝试一次，写失败也不会变成周期重试。 */
static void Test_SaveScheduling(void)
{
    Fan_Control_t control; /* 保存调度所使用的控制快照。 */
    uint32_t attempted = 0U; /* 唯一保存任务已尝试的旋转版本。 */
    uint32_t now; /* 检查长时间静止的轮询时刻。 */
    Fan_Control_Init(&control, 600U, 0U); /* 默认值尚未被用户转动。 */
    assert(Fan_Control_SaveDue(&control, 0U, 0U, &attempted, 20000U) == 0U); /* 没有旋转时不自动写空白Flash。 */
    Fan_Control_Input(&control, 1, 0U, 1U, 100U); /* 修改到700。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 3099U) == 0U); /* 不足3秒不保存。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 3100U) == 1U); /* 刚满3秒只触发一次。 */
    for (now = 3100U; now <= 23100U; now += 20U)
    {
        assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, now) == 0U); /* 即使上次写失败，静止20秒也不重复触发。 */
    }
    Fan_Control_Input(&control, 0, 0U, 1U, 24000U); /* 新的有效旋转允许对未保存目标再尝试一次。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 26999U) == 0U); /* 新旋转重新计满3秒。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 27000U) == 1U); /* 新事件可以触发新的单次尝试。 */
    assert(Fan_Control_SaveDue(&control, 700U, 1U, &attempted, 50000U) == 0U); /* 成功保存后不周期写入。 */

    Fan_Control_Init(&control, 600U, 0U); /* 独立验证旋转重新计时。 */
    attempted = 0U;
    Fan_Control_Input(&control, 1, 0U, 1U, 100U); /* 第一次旋转。 */
    Fan_Control_Input(&control, 1, 0U, 1U, 3000U); /* 在旧等待即将到期前再次旋转。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 3100U) == 0U); /* 不能沿用旧到期时刻。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 5999U) == 0U); /* 以最近旋转为准。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 6000U) == 1U); /* 最近旋转满3秒触发。 */

    Fan_Control_Init(&control, 600U, 0U); /* 独立验证转回已保存值。 */
    attempted = 0U;
    Fan_Control_Input(&control, 1, 0U, 1U, 100U); /* 暂时调高。 */
    Fan_Control_Input(&control, -1, 0U, 1U, 200U); /* 保存前又转回600。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 3200U) == 0U); /* 最终值相同不写Flash。 */
}

/** 追踪目标只消费新反馈，检查调节周期、死区和上下占空比限幅。 */
static void Test_RunningFeedback(void)
{
    Fan_Control_t control; /* 正常闭环待测状态。 */
    Fan_Feedback_t feedback = {500U, 11U, 1U}; /* 比600目标低100 RPM的新反馈。 */
    Test_Running(&control, 600U, 500U, 0U, 10U); /* 已经处于稳定运行流程。 */
    Fan_Control_Update(&control, &feedback, 99U); /* 正常闭环调节间隔尚不足100ms。 */
    assert(control.duty_permille == 500U); /* 不能提前调整。 */
    Fan_Control_Update(&control, &feedback, 100U); /* 周期虽满，99ms时收到的样本已经消费。 */
    assert(control.duty_permille == 500U); /* 不重新使用周期前的样本。 */
    feedback.sequence = 12U;
    Fan_Control_Update(&control, &feedback, 100U); /* 满100ms且真正收到新样本。 */
    assert(control.duty_permille == 520U); /* 每次最多增加2个百分点。 */
    Fan_Control_Update(&control, &feedback, 400U); /* 重复同一个样本。 */
    assert(control.duty_permille == 520U); /* 不反复根据旧反馈加速。 */
    feedback.sequence = 13U;
    Fan_Control_Update(&control, &feedback, 400U); /* 新样本允许继续一次。 */
    assert(control.duty_permille == 540U); /* 新周期继续小幅调节。 */
    feedback.rpm = 570U;
    feedback.sequence = 14U;
    Fan_Control_Update(&control, &feedback, 600U); /* 恰好在负30 RPM死区边界。 */
    assert(control.duty_permille == 540U); /* 误差边界不追涨。 */
    feedback.rpm = 630U;
    feedback.sequence = 15U;
    Fan_Control_Update(&control, &feedback, 800U); /* 恰好在正30 RPM死区边界。 */
    assert(control.duty_permille == 540U); /* 误差边界不追跌。 */
    feedback.rpm = 631U;
    feedback.sequence = 16U;
    Fan_Control_Update(&control, &feedback, 1000U); /* 超过正向允许误差。 */
    assert(control.duty_permille == 520U); /* 应减小占空比。 */
    control.duty_permille = 990U;
    feedback.rpm = 500U;
    feedback.sequence = 17U;
    Fan_Control_Update(&control, &feedback, 1200U); /* 接近100%时继续请求增加。 */
    assert(control.duty_permille == 1000U); /* 饱和到100%，不能超过。 */
    feedback.sequence = 18U;
    Fan_Control_Update(&control, &feedback, 1400U); /* 已经100%仍达不到目标。 */
    assert(control.duty_permille == 1000U); /* 无积分累积或数值回绕。 */
    control.duty_permille = 10U;
    feedback.rpm = 900U;
    feedback.sequence = 19U;
    Fan_Control_Update(&control, &feedback, 1600U); /* 小于一个步长时要求减小。 */
    assert(control.duty_permille == 0U); /* 下限0%，不能无符号下溢。 */
    feedback.sequence = 20U;
    Fan_Control_Update(&control, &feedback, 1800U); /* 下限处再次要求减小。 */
    assert(control.duty_permille == 0U); /* 继续保持0%。 */
}

/** 停机必须等待调整后的完整新周期，并达到最小下降量才继续下一步。 */
static void Test_StopFeedbackGates(void)
{
    Fan_Control_t control; /* 减速待测状态。 */
    Fan_Feedback_t feedback = {900U, 100U, 1U}; /* 第一步减速前基准。 */
    Test_Running(&control, 900U, 600U, 0U, 100U); /* 从60%输出开始停止。 */
    Fan_Control_Input(&control, 0, 1U, 0U, 10U); /* 用户按下停止。 */
    assert(Fan_Control_LedCount(&control) == 0U); /* 按键立即灭灯。 */
    Fan_Control_Update(&control, &feedback, 10U); /* 先减小第一步。 */
    assert(control.duty_permille == 580U && control.state == FAN_STATE_STOPPING); /* 第一步为58%。 */
    feedback.rpm = 880U;
    feedback.sequence = 102U;
    Fan_Control_Update(&control, &feedback, 209U); /* 虽有新下降，但最短等待时间未满。 */
    assert(control.duty_permille == 580U); /* 不过快连续调节。 */
    feedback.sequence = 100U;
    Fan_Control_Update(&control, &feedback, 210U); /* 旧序号即使数值下降也不能用。 */
    assert(control.duty_permille == 580U); /* 不接受旧周期。 */
    feedback.sequence = 101U;
    Fan_Control_Update(&control, &feedback, 210U); /* 只有一个新边沿，周期可能跨越上一步。 */
    assert(control.duty_permille == 580U); /* 必须至少相差两个捕获序号。 */
    feedback.sequence = 102U;
    feedback.rpm = 891U;
    Fan_Control_Update(&control, &feedback, 210U); /* 新完整周期只下降9 RPM。 */
    assert(control.duty_permille == 580U); /* 不能把测量微小抖动当成跟随。 */
    feedback.rpm = 890U;
    Fan_Control_Update(&control, &feedback, 210U); /* 新完整周期恰好下降10 RPM。 */
    assert(control.duty_permille == 560U && control.stop_rpm == 890U); /* 才开始下一步并重建基准。 */
    feedback.sequence = 103U;
    feedback.rpm = 880U;
    Fan_Control_Update(&control, &feedback, 410U); /* 第二步后还只有一个新边沿。 */
    assert(control.duty_permille == 560U); /* 每一步都独立要求完整新周期。 */
    feedback.sequence = 104U;
    Fan_Control_Update(&control, &feedback, 410U); /* 第二步后的新完整周期已下降10。 */
    assert(control.duty_permille == 540U); /* 继续按同样规则小步减速。 */
}

/** 检查停机1秒平台、反馈丢失、低占空比及正常运行失去测速。 */
static void Test_StopAndTachTimeouts(void)
{
    Fan_Control_t control; /* 超时场景状态。 */
    Fan_Feedback_t feedback = {900U, 100U, 1U}; /* 保持原转速的平台反馈。 */
    Test_Running(&control, 900U, 600U, 0U, 100U); /* 准备平台停机。 */
    Fan_Control_Input(&control, 0, 1U, 0U, 10U); /* 停止使能。 */
    Fan_Control_Update(&control, &feedback, 10U); /* 第一步减至58%。 */
    Fan_Control_Update(&control, &feedback, 1009U); /* 距本步999ms。 */
    assert(control.duty_permille == 580U); /* 超时前仍等待反馈。 */
    Fan_Control_Update(&control, &feedback, 1010U); /* 等待达到1秒。 */
    assert(control.duty_permille == 0U && control.state == FAN_STATE_STOPPED); /* 立即降到0，不再缓慢递减。 */
    assert((control.faults & FAN_FAULT_STOP_TIMEOUT) != 0U); /* 保留无法继续下降的原因。 */

    Test_Running(&control, 900U, 600U, 0U, 100U); /* 准备停机中TACH丢失。 */
    Fan_Control_Input(&control, 0, 1U, 0U, 10U); /* 开始停止。 */
    Fan_Control_Update(&control, &feedback, 10U); /* 建立有效减速基准。 */
    feedback.valid = 0U;
    Fan_Control_Update(&control, &feedback, 1010U); /* 一秒内没有有效下降。 */
    assert(control.duty_permille == 0U); /* 反馈丢失最终也强制0%。 */
    assert((control.faults & (FAN_FAULT_STOP_TIMEOUT | FAN_FAULT_TACH_LOST)) ==
           (FAN_FAULT_STOP_TIMEOUT | FAN_FAULT_TACH_LOST)); /* 两种异常均可观察。 */

    Test_Running(&control, 900U, 10U, 0U, 100U); /* 输出低于一个减速步长。 */
    Fan_Control_Input(&control, 0, 1U, 0U, 10U); /* 低输出时停止。 */
    Fan_Control_Update(&control, &feedback, 10U); /* 一个步骤即可归零。 */
    assert(control.duty_permille == 0U && control.state == FAN_STATE_STOPPED); /* 不出现无符号下溢。 */

    Test_Running(&control, 600U, 500U, 0U, 100U); /* 正常运行时测速丢失。 */
    Fan_Control_Update(&control, &feedback, 999U); /* 还在容忍窗口。 */
    assert(control.duty_permille == 500U); /* 不盲目提高输出。 */
    Fan_Control_Update(&control, &feedback, 1000U); /* 达到运行失效时限。 */
    assert(control.duty_permille == 0U && control.enabled == 0U); /* 失效停机并锁住旋钮。 */
    assert((control.faults & FAN_FAULT_TACH_LOST) != 0U); /* 保留测速异常。 */

    Fan_Control_Init(&control, 600U, 0U); /* 从上电软启动检查始终没有反馈。 */
    Fan_Control_Update(&control, &feedback, 0U); /* 启动第一步。 */
    Fan_Control_Update(&control, &feedback, 12000U); /* 超过最长启动等待。 */
    assert(control.duty_permille == 0U && control.enabled == 0U); /* 不因无反馈无限增加输出。 */
}

/** 检查启动和减速中快速恢复都拒绝旧valid样本，必须有调整后的新周期。 */
static void Test_FreshFeedbackAfterResume(void)
{
    Fan_Control_t control; /* 恢复流程状态。 */
    Fan_Feedback_t feedback = {1500U, 100U, 1U}; /* 保存下来的旧有效样本，数值故意高于目标。 */
    Fan_Control_Init(&control, 600U, 0U); /* 启动不能把旧valid当成风扇已经响应。 */
    Fan_Control_Update(&control, &feedback, 0U); /* 播种旧序号，软启动可先小增一步。 */
    assert(control.state == FAN_STATE_STARTING); /* 旧valid不能直接进入运行。 */
    feedback.sequence = 101U;
    Fan_Control_Update(&control, &feedback, 200U); /* 只到一个新边沿。 */
    assert(control.state == FAN_STATE_STARTING); /* 仍不把跨启动周期当成新周期。 */
    feedback.sequence = 102U;
    Fan_Control_Update(&control, &feedback, 400U); /* 两个新捕获后才取得新完整周期。 */
    assert(control.state == FAN_STATE_SWITCHING); /* 启动后先切换档位，再根据反馈进入稳定。 */

    feedback.rpm = 900U;
    feedback.sequence = 500U;
    Test_Running(&control, 900U, 600U, 0U, 500U); /* 从有占空比的减速状态快速恢复。 */
    Fan_Control_Input(&control, 0, 1U, 0U, 10U); /* 请求停止。 */
    Fan_Control_Update(&control, &feedback, 10U); /* 第一步到58%。 */
    Fan_Control_Input(&control, 0, 1U, 0U, 20U); /* 立即再次按键恢复。 */
    feedback.rpm = 1500U;
    Fan_Control_Update(&control, &feedback, 20U); /* 仍是恢复前旧valid序号。 */
    assert(control.duty_permille == 580U); /* 不能根据旧高转速立即降输出。 */
    feedback.sequence = 501U;
    Fan_Control_Update(&control, &feedback, 220U); /* 恢复后只来一个捕获。 */
    assert(control.duty_permille == 580U); /* 等待新完整周期。 */
    feedback.sequence = 502U;
    Fan_Control_Update(&control, &feedback, 420U); /* 恢复后的新完整周期到达。 */
    assert(control.duty_permille == 560U); /* 才允许按新反馈降低一次输出。 */
}

/** 毫秒tick和捕获/旋转序号回绕时，时间阈值与新周期判断仍正确。 */
static void Test_TimeAndSequenceWrap(void)
{
    Fan_Control_t control; /* 回绕场景状态。 */
    Fan_Feedback_t feedback = {900U, UINT32_MAX, 1U}; /* 减速基准在捕获序号上限。 */
    uint32_t stop_time = UINT32_MAX - 99U; /* 步骤跨越毫秒tick回绕。 */
    uint32_t rotation_time = UINT32_MAX - 1499U; /* 保存等待跨越tick回绕。 */
    uint32_t attempted = UINT32_MAX; /* 上次已尝试的旋转版本。 */
    Test_Running(&control, 900U, 600U, stop_time, UINT32_MAX); /* 建立回绕前正常状态。 */
    Fan_Control_Input(&control, 0, 1U, 0U, stop_time); /* 在tick回绕前开始减速。 */
    Fan_Control_Update(&control, &feedback, stop_time); /* 第一步建立序号FFFFFFFF基准。 */
    feedback.rpm = 880U;
    feedback.sequence = 0U;
    Fan_Control_Update(&control, &feedback, 100U); /* 时间已满200ms，但只差一个捕获。 */
    assert(control.duty_permille == 580U); /* 不把序号回绕当成大量新样本。 */
    feedback.sequence = 1U;
    Fan_Control_Update(&control, &feedback, 100U); /* 跨回绕累计两个新捕获。 */
    assert(control.duty_permille == 560U); /* 回绕后允许下一步。 */
    Fan_Control_Update(&control, &feedback, 1099U); /* 下一步仍未满1秒。 */
    assert(control.duty_permille == 560U); /* 不提前超时。 */
    Fan_Control_Update(&control, &feedback, 1100U); /* 满1秒无下一次下降。 */
    assert(control.duty_permille == 0U); /* 回绕后的超时仍精确生效。 */

    Fan_Control_Init(&control, 600U, rotation_time); /* 在tick即将回绕时旋转。 */
    control.rotation_serial = UINT32_MAX;
    Fan_Control_Input(&control, 1, 0U, 1U, rotation_time); /* 旋转版本自然回绕到0。 */
    assert(control.rotation_serial == 0U); /* 对外事件版本允许无符号回绕。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 1499U) == 0U); /* 跨回绕2999ms不能保存。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 1500U) == 1U); /* 跨回绕3000ms保存一次。 */
    assert(Fan_Control_SaveDue(&control, 600U, 1U, &attempted, 22000U) == 0U); /* 回绕后仍不会周期重试。 */
}

/** 升降速跨过半程才推进，最后一档反馈进入±50 RPM才算稳定。 */
static void Test_SwitchingStages(void)
{
    Fan_Control_t control; /* 当前目标、分段档位和PWM请求。 */
    Fan_Feedback_t feedback = {400U, 100U, 1U}; /* 升速前最近一次稳定反馈。 */
    Test_Running(&control, 400U, 500U, 0U, 100U); /* 从稳定400档开始。 */
    Fan_Control_Input(&control, 2, 0U, 1U, 10U); /* 将最终目标转到600。 */
    assert(control.state == FAN_STATE_SWITCHING && control.target_rpm == 600U); /* 切换与最终目标立即生效。 */
    assert(control.duty_permille == 500U); /* 旋钮更新目标时PWM保持原值。 */
    Fan_Control_Update(&control, &feedback, 11U); /* 同一序号不能用于重规划。 */
    assert(control.control_target_rpm == 0U); /* 等待新完整测速周期。 */
    feedback.sequence = 101U;
    Fan_Control_Update(&control, &feedback, 12U); /* 以新反馈400规划第一个500档。 */
    assert(control.control_target_rpm == 500U && control.state == FAN_STATE_SWITCHING); /* 先追踪500。 */
    feedback.sequence = 102U;
    feedback.rpm = 449U;
    Fan_Control_Update(&control, &feedback, 13U); /* 还差1 RPM到升速半程。 */
    assert(control.control_target_rpm == 500U); /* 未达到半程继续当前档。 */
    feedback.rpm = 450U;
    Fan_Control_Update(&control, &feedback, 14U); /* 同一捕获序号不能推进半程。 */
    assert(control.control_target_rpm == 500U); /* 旧样本保持阶段。 */
    feedback.sequence = 103U;
    Fan_Control_Update(&control, &feedback, 15U); /* 新的450达到半程，时间尚不足100ms。 */
    assert(control.control_target_rpm == 600U && control.state == FAN_STATE_SWITCHING); /* 中间档推进独立于PWM调整周期。 */
    feedback.sequence = 104U;
    feedback.rpm = 549U;
    Fan_Control_Update(&control, &feedback, 16U); /* 距最终600档还有51 RPM。 */
    assert(control.state == FAN_STATE_SWITCHING); /* 刚超出稳定带。 */
    feedback.sequence = 105U;
    feedback.rpm = 550U;
    Fan_Control_Update(&control, &feedback, 17U); /* 恰好到最终档下侧稳定边界。 */
    assert(control.state == FAN_STATE_STABLE); /* 稳定带包含±50的边界。 */
    feedback.sequence = 106U;
    feedback.rpm = 540U;
    Fan_Control_Update(&control, &feedback, 18U); /* 稳定后略离开±50，但仍在保护允许带内。 */
    assert(control.state == FAN_STATE_STABLE && control.speed_error_active == 0U); /* 保持稳定分类。 */

    Test_Running(&control, 600U, 500U, 0U, 100U); /* 独立检查降速镜像流程。 */
    Fan_Control_Input(&control, -2, 0U, 1U, 10U); /* 从600切换到400。 */
    feedback.sequence = 101U;
    feedback.rpm = 600U;
    Fan_Control_Update(&control, &feedback, 12U); /* 先规划500中间档。 */
    assert(control.control_target_rpm == 500U); /* 降速也逐档推进。 */
    feedback.sequence = 102U;
    feedback.rpm = 551U;
    Fan_Control_Update(&control, &feedback, 13U); /* 降速距550半程还差1 RPM。 */
    assert(control.control_target_rpm == 500U); /* 未达半程保持。 */
    feedback.sequence = 103U;
    feedback.rpm = 550U;
    Fan_Control_Update(&control, &feedback, 14U); /* 降到半程。 */
    assert(control.control_target_rpm == 400U && control.state == FAN_STATE_SWITCHING); /* 追踪最终400档。 */
    feedback.sequence = 104U;
    feedback.rpm = 451U;
    Fan_Control_Update(&control, &feedback, 15U); /* 最终档上侧尚超出51 RPM。 */
    assert(control.state == FAN_STATE_SWITCHING); /* 没有提前稳定。 */
    feedback.sequence = 105U;
    feedback.rpm = 450U;
    Fan_Control_Update(&control, &feedback, 16U); /* 到达最终档上侧稳定边界。 */
    assert(control.state == FAN_STATE_STABLE); /* 上侧边界也算稳定。 */
}

/** 允许新样本跨多档推进，中间档满5秒优先追踪最终档。 */
static void Test_SwitchingCatchupAndTimeout(void)
{
    Fan_Control_t control; /* 当前切换状态。 */
    Fan_Control_t late_sample_control; /* 复用到期前状态，独立检查半程反馈恰好在到期时到达。 */
    Fan_Feedback_t feedback = {400U, 101U, 1U}; /* 初次重新规划所用的新反馈。 */
    uint32_t now; /* 连续有效测速的模拟时刻。 */
    uint16_t duty_before_timeout; /* 到期前的PWM，用于确认旧样本不触发调节。 */
    Test_Running(&control, 400U, 500U, 0U, 100U); /* 建立原400目标。 */
    Fan_Control_Input(&control, 6, 0U, 1U, 10U); /* 一次旋转切换到1000目标。 */
    Fan_Control_Update(&control, &feedback, 10U); /* 先以400反馈进入500中间档。 */
    assert(control.control_target_rpm == 500U); /* 初步计划从相邻档开始。 */
    feedback.sequence = 102U;
    feedback.rpm = 900U;
    Fan_Control_Update(&control, &feedback, 20U); /* 一次反馈已经跨过多个半程。 */
    assert(control.control_target_rpm == 1000U && control.state == FAN_STATE_SWITCHING); /* 已追踪最终档，但900未进±50。 */

    Test_Running(&control, 400U, 500U, 0U, 100U); /* 独立模拟在中间档的低速平台。 */
    Fan_Control_Input(&control, 6, 0U, 1U, 10U); /* 最终目标改为1000，保留多个中间档。 */
    feedback.sequence = 101U;
    feedback.rpm = 400U;
    Fan_Control_Update(&control, &feedback, 10U); /* 进入中间500档。 */
    for (now = 510U; now <= 4510U; now += 500U)
    {
        ++feedback.sequence;
        Fan_Control_Update(&control, &feedback, now); /* 持续收到400 RPM新反馈，但未达450半程。 */
        assert(control.control_target_rpm == 500U && control.faults == 0U); /* 切换过程中不因偏差启动稳定保护。 */
    }
    Fan_Control_Update(&control, &feedback, 5009U); /* 距进入中间档4999ms。 */
    assert(control.control_target_rpm == 500U); /* 上限前仍保持当前档。 */
    late_sample_control = control;
    duty_before_timeout = control.duty_permille;
    Fan_Control_Update(&control, &feedback, 5010U); /* 满5秒时仍只有旧样本。 */
    assert(control.control_target_rpm == 1000U && control.state == FAN_STATE_SWITCHING); /* 到期先追最终档，仍等待稳定。 */
    assert(control.duty_permille == duty_before_timeout); /* 旧样本不能更新PWM。 */
    ++feedback.sequence;
    Fan_Control_Update(&control, &feedback, 5011U); /* 5秒后收到新有效反馈。 */
    assert(control.control_target_rpm == 1000U && control.state == FAN_STATE_SWITCHING); /* 反馈400仍未进入最终档稳定带。 */
    assert(control.speed_error_active == 0U && control.faults == 0U); /* 切换超时仅推进档位，不触发转速偏差保护。 */

    feedback.rpm = 450U;
    Fan_Control_Update(&late_sample_control, &feedback, 5010U); /* 新半程样本恰好在5秒边界到达。 */
    assert(late_sample_control.control_target_rpm == 1000U); /* 到期优先，跳过原本的下一个600档。 */
    assert(late_sample_control.state == FAN_STATE_SWITCHING); /* 450尚不能确认最终档稳定。 */
    ++feedback.sequence;
    feedback.rpm = 950U;
    Fan_Control_Update(&late_sample_control, &feedback, 5011U); /* 到期后的新反馈进入最终档±50。 */
    assert(late_sample_control.state == FAN_STATE_STABLE); /* 用最终档的新有效样本确认稳定。 */
}

/** 保护允许带包含上下边界，超出后仅由持续2秒的新有效反馈触发。 */
static void Test_StableSpeedDeviation(void)
{
    Fan_Control_t control; /* 稳定保护及停止状态。 */
    Fan_Feedback_t feedback = {500U, 101U, 1U}; /* 600档允许带下边界。 */
    uint16_t duty_before_fault; /* 故障前PWM，用于确认复用2%减速首步。 */
    Test_Running(&control, 600U, 600U, 0U, 100U); /* 600档按最小100 RPM保护带。 */
    Fan_Control_Update(&control, &feedback, 10U); /* 精确在下边界500。 */
    assert(control.speed_error_active == 0U); /* 边界在允许范围内。 */
    feedback.sequence = 102U;
    feedback.rpm = 700U;
    Fan_Control_Update(&control, &feedback, 20U); /* 精确在上边界700。 */
    assert(control.speed_error_active == 0U); /* 上边界也在允许范围内。 */
    feedback.sequence = 103U;
    feedback.rpm = 701U;
    Fan_Control_Update(&control, &feedback, 30U); /* 超过上边界1 RPM才开始计时。 */
    assert(control.speed_error_active != 0U && control.enabled == 1U); /* 还未到保护延时。 */
    ++feedback.sequence;
    Fan_Control_Update(&control, &feedback, 2030U - 1U); /* 偏差连续1999ms。 */
    assert(control.enabled == 1U); /* 不能提前停机。 */
    duty_before_fault = control.duty_permille;
    Fan_Control_Update(&control, &feedback, 2030U); /* 计时满2秒，但没有新反馈。 */
    assert(control.enabled == 1U); /* 旧样本不能单独触发保护。 */
    ++feedback.sequence;
    Fan_Control_Update(&control, &feedback, 2030U); /* 新反馈确认偏差持续满2秒。 */
    assert((control.faults & FAN_FAULT_SPEED_DEVIATION) != 0U); /* 保留稳定转速偏差故障。 */
    assert(control.enabled == 0U && control.target_rpm == 600U); /* 保护锁旋钮并保留设定。 */
    assert(Fan_Control_LedCount(&control) == 0U); /* 故障触发时立即灭灯。 */
    assert(control.state == FAN_STATE_STOPPING && control.duty_permille == duty_before_fault - 20U); /* 走与按键停止相同的首步。 */
    Fan_Control_Update(&control, &feedback, 3030U); /* 停止一步后1秒没有有效下降。 */
    assert(control.duty_permille == 0U && control.state == FAN_STATE_STOPPED); /* 保护减速仍有0%兜底。 */
    Fan_Control_Input(&control, 0, 1U, 0U, 3040U); /* 再次按键从保存目标恢复。 */
    assert(control.enabled == 1U && control.target_rpm == 600U && control.faults == 0U); /* 故障恢复保留目标并清本次故障。 */

    Test_Running(&control, 1500U, 700U, 0U, 200U); /* 1500档采用10%即150 RPM。 */
    feedback.sequence = 201U;
    feedback.rpm = 1650U;
    Fan_Control_Update(&control, &feedback, 10U); /* 精确在10%上边界。 */
    assert(control.speed_error_active == 0U); /* 不按固定100 RPM误触发。 */
    feedback.sequence = 202U;
    feedback.rpm = 1350U;
    Fan_Control_Update(&control, &feedback, 20U); /* 精确在10%下边界。 */
    assert(control.speed_error_active == 0U); /* 下边界也不开始计时。 */
    feedback.sequence = 203U;
    feedback.rpm = 1349U;
    Fan_Control_Update(&control, &feedback, 30U); /* 低于下边界1 RPM。 */
    assert(control.speed_error_active != 0U); /* 欠速与超速都受保护。 */
    feedback.sequence = 204U;
    Fan_Control_Update(&control, &feedback, 2030U); /* 新反馈确认连续欠速2秒。 */
    assert(control.state == FAN_STATE_STOPPING && control.enabled == 0U); /* 欠速同样走停止状态机。 */
}

/** 重新进允许带、无效反馈、新目标都会清除稳定偏差累计。 */
static void Test_DeviationResets(void)
{
    Fan_Control_t control; /* 测试偏差计时复位。 */
    Fan_Feedback_t feedback = {701U, 101U, 1U}; /* 第一次超出600档允许带。 */
    Test_Running(&control, 600U, 500U, 0U, 100U); /* 从稳定600档开始。 */
    Fan_Control_Update(&control, &feedback, 10U); /* 开始第一次累计。 */
    assert(control.speed_error_active != 0U); /* 明确保护累计已经开始。 */
    ++feedback.sequence;
    feedback.rpm = 700U;
    Fan_Control_Update(&control, &feedback, 1000U); /* 返回允许带边界。 */
    assert(control.speed_error_active == 0U); /* 清除上一次累计。 */
    ++feedback.sequence;
    feedback.rpm = 701U;
    Fan_Control_Update(&control, &feedback, 1010U); /* 再次超出带宽，从零重新累计。 */
    ++feedback.sequence;
    Fan_Control_Update(&control, &feedback, 2010U); /* 距第一次已2秒，但距本次只有1秒。 */
    assert(control.enabled == 1U); /* 两段超差时长不能累计到一起。 */
    feedback.valid = 0U;
    Fan_Control_Update(&control, &feedback, 2050U); /* 无效反馈打断超差的连续性。 */
    assert(control.speed_error_active == 0U); /* 无效样本清计时。 */
    ++feedback.sequence;
    feedback.valid = 1U;
    Fan_Control_Update(&control, &feedback, 2060U); /* 新的有效超差样本重新开始累计。 */
    assert(control.speed_error_active != 0U); /* 已再次开始保护计时。 */
    Fan_Control_Input(&control, 2, 0U, 1U, 2070U); /* 切换目标到800。 */
    assert(control.state == FAN_STATE_SWITCHING && control.speed_error_active == 0U); /* 过渡期间不继承稳定档计时。 */
}

/** 主机测试入口，验证用户确认的输入、指示、保存、闭环及停止安全规则。 */
int main(void)
{
    Test_InitAndLeds(); /* 默认和LED全部阈值。 */
    Test_InputsAndPause(); /* 启停、锁旋钮和0档语义。 */
    Test_SaveScheduling(); /* 3秒单次保存和失败无周期重试。 */
    Test_RunningFeedback(); /* 新反馈、调节限幅和死区。 */
    Test_StopFeedbackGates(); /* 每步必须有完整新周期且转速下降。 */
    Test_StopAndTachTimeouts(); /* 一秒超时直接0及测速异常。 */
    Test_FreshFeedbackAfterResume(); /* 快速恢复不能消费旧valid反馈。 */
    Test_TimeAndSequenceWrap(); /* 时间和事件序号回绕。 */
    Test_SwitchingStages(); /* 升降速半程、最终档与稳定带。 */
    Test_SwitchingCatchupAndTimeout(); /* 多级跨越与中间档5秒处理。 */
    Test_StableSpeedDeviation(); /* 保护边界、连续时间、逐步停止和恢复。 */
    Test_DeviationResets(); /* 超差计时的三个复位条件。 */
    puts("fan_control: inputs/save/feedback/staged targets/stable protection/stop/wrap passed"); /* 输出实际测试结果。 */
    return 0;
}
