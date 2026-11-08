#ifndef FAN_CONTROL_H
#define FAN_CONTROL_H

#include <stdint.h> /* 使用固定宽度的转速、时间和状态类型。 */

#define FAN_RPM_MAX              1500U /* 用户允许的目标转速上限，单位RPM。 */
#define FAN_RPM_STEP              100U /* 编码器每个机械格对应的目标步进。 */
#define FAN_RPM_DEFAULT           600U /* Flash无有效记录时的默认目标。 */
#define FAN_FAULT_STOP_TIMEOUT   0x01U /* 减速一步后反馈未跟随，已强制请求0%。 */
#define FAN_FAULT_TACH_LOST      0x02U /* 启动或运行过程中未取得有效测速。 */
#define FAN_FAULT_SPEED_DEVIATION 0x04U /* 稳定运行后转速持续超出允许范围。 */
#define FAN_FAULT_STAGE_TIMEOUT  0x08U /* 中间档5秒未过半程，已保护归零。 */

/* 控制状态表示软件输出流程，不把0%请求等同于实物已经停转。 */
typedef enum
{
    FAN_STATE_STOPPED = 0, /* 已输出风扇端0%请求。 */
    FAN_STATE_STARTING,    /* 启动切换阶段，先建立有效测速。 */
    FAN_STATE_SWITCHING,   /* 分段追踪，等待反馈接近最终目标。 */
    FAN_STATE_STOPPING,    /* 每次减小后等待实际转速下降。 */
    FAN_STATE_STABLE       /* 已进入最终目标附近，开始监测持续超差。 */
} Fan_State_t;

/* 硬件层发布的一致测速快照；序号只用于区分新旧周期。 */
typedef struct
{
    uint32_t rpm;      /* 最新完整周期换算的转速。 */
    uint32_t sequence; /* 捕获样本序号，支持无符号回绕。 */
    uint8_t valid;     /* 当前样本是否仍在有效时限内。 */
} Fan_Feedback_t;

/* 状态仅由Task1修改，其他任务通过user_work中的一致快照读取。 */
typedef struct
{
    uint16_t target_rpm;       /* 当前运行目标，按键停机时保留，指定保护停机时归零。 */
    uint16_t save_target_rpm;  /* 人工选择的目标，保护归零不覆盖待保存值。 */
    uint16_t control_target_rpm; /* 当前闭环追踪的中间或最终目标，0表示等待重新规划。 */
    uint16_t duty_permille;    /* 风扇端请求占空比，范围0～1000。 */
    uint8_t enabled;           /* 按键运行使能；0RPM档仍可保持使能。 */
    uint8_t faults;            /* 本次运行的控制异常位。 */
    Fan_State_t state;         /* 当前启动、切换、稳定或停机状态。 */
    uint32_t rotation_ms;      /* 最近一次运行状态下的旋转时刻。 */
    uint32_t rotation_serial;  /* 旋转事件版本，供保存任务只尝试一次。 */
    uint32_t step_ms;          /* 最近一次占空比调整时刻。 */
    uint32_t start_ms;         /* 本次软启动起始时刻。 */
    uint32_t last_valid_ms;    /* 最近一次取得有效测速的时刻。 */
    uint32_t sample_sequence;  /* 已消费的测速序号。 */
    uint32_t start_sequence;   /* 软启动后首个采样序号，拒绝旧的有效反馈。 */
    uint8_t start_seeded;      /* 当前软启动是否建立过捕获序号基准。 */
    uint32_t stop_sequence;    /* 当前减速步骤开始前的测速序号。 */
    uint32_t stop_rpm;         /* 当前减速步骤的比较基准转速。 */
    uint8_t stop_reference;    /* 当前减速步骤是否有有效基准。 */
    uint16_t stage_threshold_rpm; /* 当前中间档推进所需的半程转速。 */
    uint32_t transition_step_ms;  /* 当前中间档开始等待的时刻。 */
    uint32_t speed_error_since_ms; /* 首次超出异常范围的测速时刻。 */
    uint8_t speed_error_active;    /* 是否正在累计连续超差时间。 */
} Fan_Control_t;

/* 在user_Init中建立开机自动运行状态，不读写外设。 */
void Fan_Control_Init(Fan_Control_t *control, uint16_t target_rpm, uint32_t now_ms);
/* 应用旋转与按键事件；暂停时仍拒绝一切旋转事件。 */
void Fan_Control_Input(Fan_Control_t *control, int16_t steps,
                       uint8_t button_pressed, uint8_t rotated, uint32_t now_ms);
/* 消费新测速快照并推进有时间限制的非阻塞控制流程。 */
void Fan_Control_Update(Fan_Control_t *control, const Fan_Feedback_t *feedback,
                        uint32_t now_ms);
/* 根据运行使能及目标阈值返回需要点亮的累计灯数。 */
uint8_t Fan_Control_LedCount(const Fan_Control_t *control);
/* 旋转静止3秒且目标变化后仅发起一次保存尝试，不按周期反复擦写。 */
uint8_t Fan_Control_SaveDue(const Fan_Control_t *control, uint16_t saved_rpm,
                            uint8_t saved_valid, uint32_t *attempt_serial,
                            uint32_t now_ms);

#endif
