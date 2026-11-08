/* 任务组织：控制、硬件采集与Flash存储分别由独立模块完成。 */
#include "user_work.h" /* 用户任务入口及调试快照。 */
#include "fan_io.h" /* 编码器、按键、测速、PWM和LED硬件接口。 */
#include "fan_settings.h" /* 双页设置日志。 */
#include "cmsis_os.h" /* 现有RTOS任务等待接口。 */
#include "FreeRTOS.h" /* 短临界区及静态队列支持。 */
#include "queue.h" /* 输入任务到控制任务的单向事件队列。 */
#include "main.h" /* HAL毫秒时基。 */

#define USER_INPUT_QUEUE_LENGTH 16U /* 最多缓存16次输入采样。 */

/* 保留输入时刻和启停代次，防止暂停期间的旧消息在恢复后重放。 */
typedef struct
{
    Fan_Input_t input; /* 本次旋转和去抖后的按键事件。 */
    uint32_t time_ms; /* 实际采样时间。 */
    uint32_t epoch; /* 采样时控制任务发布的启停代次。 */
} User_InputEvent_t;

volatile Fan_AppStatus_t g_fan_status; /* Keil可观察的一致业务快照。 */
static Fan_Control_t user_control; /* 只有Task1修改的运行控制状态。 */
static StaticQueue_t user_queue_control; /* 不占RTOS动态堆的队列控制块。 */
static uint8_t user_queue_storage[USER_INPUT_QUEUE_LENGTH * sizeof(User_InputEvent_t)]; /* 静态队列数据区。 */
static QueueHandle_t user_input_queue; /* Task2生产、Task1消费的唯一输入队列。 */
static volatile uint8_t user_application_ready; /* 初始化完成后才放行其他任务。 */
static uint8_t user_hardware_ok; /* 定时器业务是否全部启动成功。 */
static uint16_t user_saved_rpm; /* 最近成功保存的目标，初始化建立、Task3后续更新。 */
static uint8_t user_saved_valid; /* Flash是否具有有效记录。 */
static uint8_t user_storage_error; /* 最近一次保存失败状态。 */
static uint32_t user_input_overflows; /* 输入队列异常满计数，正常调度应为0。 */
static uint32_t user_input_epoch; /* Task1每次启停切换后递增的消息代次。 */

/* 多字段复制期间使用短临界区，避免新旧状态混读。 */
static void User_ReadStatus(Fan_AppStatus_t *status)
{
    taskENTER_CRITICAL(); /* 保护完整快照读取。 */
    *status = g_fan_status;
    taskEXIT_CRITICAL(); /* 立即恢复调度。 */
}

/* 把最新快照、输入队列及3秒判定作为一次原子检查；擦写不在临界区内。 */
static uint8_t User_TakeSaveRequest(Fan_AppStatus_t *status, uint32_t *attempt_serial)
{
    uint8_t due = 0U; /* 本次是否取得一个需要实际执行的保存请求。 */

    taskENTER_CRITICAL(); /* 防止Task1在复制与判定之间发布新的旋转状态。 */
    *status = g_fan_status;
    if ((status->hardware_ok != 0U) && (uxQueueMessagesWaiting(user_input_queue) == 0U)) /* 有待消费旋转时不能提前保存。 */
    {
        due = Fan_Control_SaveDue(&status->control, status->saved_rpm,
                                   status->saved_valid, attempt_serial,
                                   HAL_GetTick()); /* 同一快照上完成单次保存资格判定。 */
    }
    taskEXIT_CRITICAL(); /* 退出后才允许执行耗时Flash操作。 */
    return due;
}

/* Task1发布完整运行状态，存储任务仅回报独立的保存字段。 */
static void User_PublishStatus(const Fan_Tach_t *tach)
{
    taskENTER_CRITICAL(); /* 发布期间不执行任何Flash操作。 */
    g_fan_status.control = user_control;
    g_fan_status.measured_rpm = tach->rpm;
    g_fan_status.tach_valid = tach->valid;
    g_fan_status.hardware_ok = user_hardware_ok;
    g_fan_status.saved_rpm = user_saved_rpm;
    g_fan_status.saved_valid = user_saved_valid;
    g_fan_status.storage_error = user_storage_error;
    g_fan_status.input_overflows = user_input_overflows;
    g_fan_status.input_epoch = user_input_epoch;
    taskEXIT_CRITICAL(); /* 完成发布后立即退出。 */
}

/* 在user_Init完成前阻塞，不让输入任务使用未建立的队列。 */
static void User_WaitReady(void)
{
    while (user_application_ready == 0U)
    {
        osDelay(1U); /* 等待就绪并让出处理器。 */
    }
    __DMB(); /* 就绪之后才读取已经初始化的状态。 */
}

/* 参考原F407组织方式，由默认任务在常驻业务循环前调用一次。 */
void user_Init(void)
{
    uint16_t target = FAN_RPM_DEFAULT; /* 无有效记录时默认600RPM。 */
    Fan_Tach_t tach = {0U, 0U, 0U}; /* 初始化时不伪造实际转速。 */

    if (user_application_ready != 0U)
    {
        return;
    }
    user_saved_valid = Fan_Settings_Load(&target); /* 启动只读恢复目标，不自动写默认值。 */
    user_saved_rpm = target;
    user_storage_error = 0U;
    user_input_overflows = 0U;
    user_input_epoch = 0U;
    user_input_queue = xQueueCreateStatic(USER_INPUT_QUEUE_LENGTH,
                                          sizeof(User_InputEvent_t),
                                          user_queue_storage,
                                          &user_queue_control); /* 建立唯一输入事件通道。 */
    Fan_Control_Init(&user_control, target, HAL_GetTick()); /* 上电自动使能，0档仍是可调状态。 */
    user_hardware_ok = Fan_IO_Init(); /* 安全比较值建立后启动PWM、编码器和测速。 */
    if ((user_hardware_ok == 0U) || (user_input_queue == 0))
    {
        user_hardware_ok = 0U;
        user_control.enabled = 0U;
        user_control.state = FAN_STATE_STOPPED;
        Fan_IO_SetDuty(0U); /* 初始化失败时保持软件能够发出的0%请求。 */
    }
    User_PublishStatus(&tach); /* 初始化完成后建立首次状态快照。 */
    __DMB(); /* 先发布数据，再放行输入与存储任务。 */
    user_application_ready = 1U;
}

/* Task1独占目标、启停状态和PWM，短周期推进非阻塞控制流程。 */
void My_Task1(void)
{
    User_InputEvent_t event; /* 当前正在处理的输入事件。 */
    Fan_Tach_t tach; /* 当前一致硬件测速快照。 */
    Fan_Feedback_t feedback; /* 提交给纯控制模块的测速数据。 */
    uint32_t now_ms; /* 当前HAL毫秒时刻。 */
    uint8_t was_enabled; /* 更新前使能状态，用于异常停机后作废旧输入。 */

    user_Init(); /* 单次初始化入口放在常驻循环之前。 */
    for (;;)
    {
        if (user_hardware_ok != 0U)
        {
            while (xQueueReceive(user_input_queue, &event, 0U) == pdPASS) /* 按发生顺序处理输入。 */
            {
                if ((event.input.button_pressed != 0U) || (event.epoch == user_input_epoch))
                {
                    Fan_Control_Input(&user_control, event.input.steps,
                                       event.input.button_pressed, event.input.rotated,
                                       event.time_ms); /* 控制任务再次检查暂停状态。 */
                    if (event.input.button_pressed != 0U)
                    {
                        user_input_epoch++; /* 任何启停切换都作废此前采样的旋转。 */
                    }
                }
            }
            now_ms = HAL_GetTick(); /* 获取本轮控制时刻。 */
            Fan_IO_GetTach(now_ms, &tach); /* RPM换算在任务中完成。 */
            feedback.rpm = tach.rpm;
            feedback.sequence = tach.sequence;
            feedback.valid = tach.valid;
            was_enabled = user_control.enabled;
            Fan_Control_Update(&user_control, &feedback, now_ms); /* 软启动、闭环及反馈门控减速。 */
            if (was_enabled != user_control.enabled)
            {
                user_input_epoch++; /* 异常锁停也禁止重放旧旋转事件。 */
            }
            Fan_IO_SetDuty(user_control.duty_permille); /* 唯一运行时PWM输出点，内部完成反相。 */
            User_PublishStatus(&tach); /* 提供一致的运行与保存判定状态。 */
        }
        osDelay(10U); /* 阻塞约10ms，避免无等待空转。 */
    }
}

/* Task2读取硬件编码器累计计数并去抖按键，不直接改目标或PWM。 */
void My_Task2(void)
{
    Fan_AppStatus_t status; /* 控制任务发布的运行使能及消息代次。 */
    User_InputEvent_t event; /* 本次采样的输入消息。 */
    uint32_t last_epoch = 0U; /* 输入任务上次观察到的启停代次。 */
    uint8_t allow_rotation; /* 本次是否允许输出旋转步进。 */

    User_WaitReady(); /* 等待配置、驱动及静态队列建立。 */
    for (;;)
    {
        User_ReadStatus(&status); /* 使用一致控制状态决定是否接受旋转。 */
        if (status.hardware_ok != 0U)
        {
            event.time_ms = HAL_GetTick(); /* 记录采样的实际时间。 */
            event.epoch = status.input_epoch;
            allow_rotation = ((status.control.enabled != 0U) &&
                              (last_epoch == status.input_epoch)) ? 1U : 0U;
            last_epoch = status.input_epoch;
            Fan_IO_PollInput(event.time_ms, allow_rotation, &event.input); /* 暂停或代次切换时清半格残留。 */
            if ((event.input.rotated != 0U) || (event.input.button_pressed != 0U))
            {
                if (xQueueSend(user_input_queue, &event, 0U) != pdPASS) /* 无阻塞提交输入。 */
                {
                    taskENTER_CRITICAL(); /* 只记录异常，不绕过控制任务改变输出。 */
                    user_input_overflows++;
                    taskEXIT_CRITICAL(); /* 恢复调度。 */
                }
            }
        }
        osDelay(5U); /* 短周期读取计数差与按键电平。 */
    }
}

/* Task3累计显示目标灯条，且是运行时唯一Flash写入任务。 */
void My_Task3(void)
{
    Fan_AppStatus_t status; /* 当前显示及保存使用的完整快照。 */
    uint32_t attempt_serial = 0U; /* 已尝试保存的旋转版本，避免静止时重复写入。 */
    uint8_t success; /* 本次日志提交实际结果。 */

    User_WaitReady(); /* 初始化只读恢复完成后才显示或保存。 */
    for (;;)
    {
        User_ReadStatus(&status); /* 复制结束立即退出临界区。 */
        Fan_IO_SetLeds(Fan_Control_LedCount(&status.control)); /* 按目标显示，按键停止全灭。 */
        if (User_TakeSaveRequest(&status, &attempt_serial) != 0U) /* 对最新状态原子检查，只触发一次。 */
        {
            Fan_IO_FlashBegin(); /* 使旧测速失效，硬件PWM和编码器仍继续运行。 */
            success = Fan_Settings_Save(status.control.target_rpm); /* 唯一运行时Flash保存调用。 */
            Fan_IO_FlashEnd(); /* 丢弃跨擦写周期，等待新的完整捕获周期。 */
            taskENTER_CRITICAL(); /* 回报已保存值，不清除期间可能更新的用户目标。 */
            if (success != 0U)
            {
                user_saved_rpm = status.control.target_rpm;
                user_saved_valid = 1U;
            }
            user_storage_error = (success == 0U) ? 1U : 0U;
            taskEXIT_CRITICAL(); /* 失败不周期重试，下一次有效旋转后才允许再次尝试。 */
        }
        osDelay(20U); /* 低频显示/判定，不按20ms或3s周期反复写Flash。 */
    }
}
