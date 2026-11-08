/* 风扇板硬件采集与输出；初始化参数继续由CubeMX生成文件维护。 */
#include "fan_io.h" /* 引入本模块业务接口及可校准参数。 */
#include "main.h"   /* 使用现有GPIO位号及STM32 HAL声明。 */
#include "tim.h"    /* 使用CubeMX建立的TIM1、TIM2、TIM3句柄。 */

#define FAN_BUTTON_DEBOUNCE_MS 30U    /* 按下与释放均稳定30ms后才更新状态。 */
#define FAN_TACH_TIMER_HZ      10000U /* TIM3当前72MHz/(7199+1)，每计数0.1ms。 */
#define FAN_TACH_MIN_TICKS     100U   /* 小于10ms的周期视为毛刺，不当作正常转速。 */
#define FAN_TACH_MAX_TICKS     10000U /* 超过1s的完整周期不作为可信反馈。 */

static uint16_t fan_encoder_previous;    /* 上次已消费的TIM2计数器值。 */
static int32_t fan_encoder_remainder;    /* 不足一个机械格的残余计数。 */
static uint8_t fan_button_candidate;     /* 当前待去抖按键电平状态，1表示按下。 */
static uint8_t fan_button_stable;        /* 上次已经稳定确认的按键状态。 */
static uint32_t fan_button_changed_ms;   /* 候选按键状态开始稳定的时间。 */

static volatile uint16_t fan_tach_previous; /* 上次可信捕获/播种的TIM3计数值。 */
static volatile uint32_t fan_tach_last_ms;  /* 上次可信捕获的HAL毫秒时刻。 */
static volatile uint32_t fan_tach_period;   /* 最新有效完整脉冲周期计数。 */
static volatile uint32_t fan_tach_sequence; /* 单调递增有效周期序号，允许自然回绕。 */
static volatile uint8_t fan_tach_seeded;    /* 已保存首个捕获，下一次才能计算周期。 */
static volatile uint8_t fan_tach_valid;     /* ISR发布的当前周期有效标记。 */
static volatile uint8_t fan_tach_suspended; /* Flash期间禁止接受捕获。 */
static uint8_t fan_tach_irq_was_enabled;    /* Flash之前捕获中断的使能状态。 */

/* 恢复进入短临界区前的中断屏蔽状态，不改变调用者原有状态。 */
static void Fan_IO_Unlock(uint32_t primask)
{
    __DMB(); /* 发布短临界区中的共享状态写入。 */
    __set_PRIMASK(primask); /* 恢复原有全局中断状态。 */
}

/* 设置风扇端占空比；风扇0%时PA8保持高，不能关闭PWM通道。 */
void Fan_IO_SetDuty(uint16_t permille)
{
    uint32_t period; /* 一个完整PWM周期的定时器计数。 */
    uint32_t compare; /* 经NPN反相后的MCU高电平计数。 */

    if (permille > 1000U)
    {
        permille = 1000U;
    }
    period = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1U; /* 读取生成配置中的真实周期。 */
    compare = period * (1000U - permille) / 1000U;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare); /* 保持通道运行，仅更新CCR1。 */
}

/* 从LED5到LED1按PB1/PB15/PB14/PB13/PB12顺序累计点亮，低电平有效。 */
void Fan_IO_SetLeds(uint8_t count)
{
    uint16_t active = 0U; /* 本次需拉低点亮的引脚掩码。 */
    const uint16_t all = LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin | LED5_Pin; /* 全部业务LED引脚。 */

    if (count >= 1U) { active |= LED5_Pin; }
    if (count >= 2U) { active |= LED4_Pin; }
    if (count >= 3U) { active |= LED3_Pin; }
    if (count >= 4U) { active |= LED2_Pin; }
    if (count >= 5U) { active |= LED1_Pin; }
    GPIOB->BSRR = ((uint32_t)active << 16U) | ((uint32_t)all & ~(uint32_t)active);
}

/* 仅启动既有硬件，不重写时钟、GPIO、定时器或NVIC初始化。 */
uint8_t Fan_IO_Init(void)
{
    uint32_t primask = __get_PRIMASK(); /* 保存初始化短临界区前的中断状态。 */

    __disable_irq(); /* 清空ISR共享状态时保持一致性。 */
    fan_tach_previous = 0U;
    fan_tach_last_ms = 0U;
    fan_tach_period = 0U;
    fan_tach_sequence = 0U;
    fan_tach_seeded = 0U;
    fan_tach_valid = 0U;
    fan_tach_suspended = 0U;
    fan_tach_irq_was_enabled = 0U;
    Fan_IO_Unlock(primask); /* 清空共享状态后立即恢复中断。 */

    fan_encoder_previous = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2); /* 记录首个硬件计数基准。 */
    fan_encoder_remainder = 0;
    fan_button_candidate = (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_RESET) ? 1U : 0U; /* 读取初始按键电平。 */
    fan_button_stable = fan_button_candidate; /* 上电已经按住时，须先释放再按下才产生事件。 */
    fan_button_changed_ms = HAL_GetTick(); /* 去抖统一使用HAL毫秒时基。 */
    Fan_IO_SetLeds(0U); /* 业务启动前先熄灭所有LED。 */
    Fan_IO_SetDuty(0U); /* PWM使能前先将CCR设为ARR+1，避免错误全速脉冲。 */
    htim1.Instance->EGR = TIM_EGR_UG; /* 先提交CCR预装载值，避免首个周期仍采用生成时的Pulse=0。 */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) /* 启动持续输出的TIM1 PWM。 */
    {
        return 0U;
    }
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) /* 由TIM2硬件累计A/B相计数，无需边沿中断。 */
    {
        return 0U;
    }
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1 | TIM_FLAG_CC1OF); /* 清除业务启动前遗留的捕获状态。 */
    if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK) /* 启动现有TIM3下降沿捕获。 */
    {
        return 0U;
    }
    return 1U;
}

/* 从硬件累计值提取机械格数；暂停时丢弃变化而不是停止更新基准。 */
void Fan_IO_PollInput(uint32_t now_ms, uint8_t allow_rotation, Fan_Input_t *input)
{
    uint16_t current; /* 本次TIM2硬件计数。 */
    int32_t delta; /* 已消除16位自然回绕的增量。 */
    int32_t detents; /* 本次已经走完的机械格数。 */
    uint8_t pressed; /* 原始按键是否处于低电平按下。 */

    if (input == 0)
    {
        return;
    }
    input->steps = 0;
    input->button_pressed = 0U;
    input->rotated = 0U;
    current = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2); /* 读取硬件已经完成的正交计数。 */
    delta = (int32_t)current - (int32_t)fan_encoder_previous;
    fan_encoder_previous = current;
    if (delta > 32767) { delta -= 65536; }
    if (delta < -32768) { delta += 65536; }

    pressed = (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_RESET) ? 1U : 0U; /* 采样低有效按键。 */
    if (pressed != fan_button_candidate)
    {
        fan_button_candidate = pressed;
        fan_button_changed_ms = now_ms;
    }
    else if ((pressed != fan_button_stable) && ((uint32_t)(now_ms - fan_button_changed_ms) >= FAN_BUTTON_DEBOUNCE_MS))
    {
        fan_button_stable = pressed;
        if (pressed != 0U)
        {
            input->button_pressed = 1U;
        }
    }
    if ((allow_rotation == 0U) || (input->button_pressed != 0U))
    {
        fan_encoder_remainder = 0; /* 停止与启停切换时清除半格及当次旋转。 */
        return;
    }
    fan_encoder_remainder += delta * FAN_ENCODER_DIRECTION;
    input->rotated = (delta != 0) ? 1U : 0U; /* 不足一格或达到目标限位时，也算仍在旋转。 */
    detents = fan_encoder_remainder / FAN_ENCODER_COUNTS_PER_DETENT;
    fan_encoder_remainder -= detents * FAN_ENCODER_COUNTS_PER_DETENT;
    input->steps = (int16_t)detents;
}

/* 只复制共享字段时短暂屏蔽中断，除法计算留在任务上下文。 */
void Fan_IO_GetTach(uint32_t now_ms, Fan_Tach_t *out)
{
    uint32_t period; /* 最新完整捕获周期。 */
    uint32_t last_ms; /* 该周期末端的HAL毫秒时刻。 */
    uint32_t primask; /* 调用者原有中断屏蔽状态。 */
    uint8_t valid; /* 与周期和序号一致的有效标志。 */

    if (out == 0)
    {
        return;
    }
    primask = __get_PRIMASK(); /* 保存原中断状态。 */
    __disable_irq(); /* 防止ISR在复制多字段中途发布新样本。 */
    period = fan_tach_period;
    last_ms = fan_tach_last_ms;
    out->sequence = fan_tach_sequence;
    valid = fan_tach_valid;
    Fan_IO_Unlock(primask); /* 完成快照后立即恢复中断。 */
    out->rpm = 0U;
    out->valid = 0U;
    if ((valid != 0U) && (period >= FAN_TACH_MIN_TICKS) && (period <= FAN_TACH_MAX_TICKS)
        && ((uint32_t)(now_ms - last_ms) < FAN_TACH_TIMEOUT_MS))
    {
        out->rpm = (60U * FAN_TACH_TIMER_HZ) / (period * FAN_TACH_PULSES_PER_REV);
        out->valid = 1U;
    }
}

/* TIM3捕获回调仅记录时间戳；不执行转速除法、控制输出或Flash写入。 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint16_t capture; /* 当前硬件锁存的16位捕获值。 */
    uint16_t period; /* 两个捕获间模65536的周期，时基为0.1ms。 */
    uint32_t now_ms; /* 当前捕获到达时的HAL毫秒时刻。 */

    if ((htim->Instance != TIM3) || (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1))
    {
        return;
    }
    capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); /* 读取本次硬件捕获，不用当前CNT替代。 */
    now_ms = HAL_GetTick(); /* 用独立HAL时基排除长期无脉冲后的模回绕假周期。 */
    if ((fan_tach_suspended != 0U) || (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1OF) != RESET)) /* 过捕获意味着中间边沿丢失。 */
    {
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1OF); /* 清除过捕获状态，下一次重新播种。 */
        fan_tach_seeded = 0U;
        fan_tach_valid = 0U;
        return;
    }
    if ((fan_tach_seeded == 0U) || ((uint32_t)(now_ms - fan_tach_last_ms) >= FAN_TACH_TIMEOUT_MS))
    {
        fan_tach_previous = capture;
        fan_tach_last_ms = now_ms;
        fan_tach_seeded = 1U;
        fan_tach_valid = 0U;
        return; /* 首个边沿只建立基准，不虚构完整周期。 */
    }
    period = (uint16_t)(capture - fan_tach_previous);
    if (period < FAN_TACH_MIN_TICKS)
    {
        return; /* 太短的毛刺不移动可信捕获基准，以免污染下一正常周期。 */
    }
    fan_tach_previous = capture;
    fan_tach_last_ms = now_ms;
    if (period > FAN_TACH_MAX_TICKS)
    {
        fan_tach_valid = 0U;
        return;
    }
    fan_tach_period = period;
    fan_tach_sequence++;
    __DMB(); /* 在有效标记前发布本次完整周期和序号。 */
    fan_tach_valid = 1U;
}

/* 保存前只关闭TIM3捕获中断源，PWM和编码器硬件计数继续运行。 */
void Fan_IO_FlashBegin(void)
{
    uint32_t primask = __get_PRIMASK(); /* 只保护几条捕获状态写入指令。 */

    __disable_irq(); /* 避免捕获ISR与暂停状态更新交错。 */
    fan_tach_irq_was_enabled = ((htim3.Instance->DIER & TIM_IT_CC1) != 0U) ? 1U : 0U;
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1); /* 仅屏蔽TIM3捕获，不屏蔽系统定时器。 */
    fan_tach_suspended = 1U;
    fan_tach_seeded = 0U;
    fan_tach_valid = 0U;
    Fan_IO_Unlock(primask); /* 擦写实际耗时期间不长期关闭全局中断。 */
}

/* 恢复前清掉擦写期间积累的捕获状态，首个新边沿仅重新建立基准。 */
void Fan_IO_FlashEnd(void)
{
    uint32_t primask = __get_PRIMASK(); /* 保存短临界区前的中断状态。 */

    __disable_irq(); /* 原子恢复捕获状态和中断源。 */
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1 | TIM_FLAG_CC1OF); /* 丢弃跨擦写捕获及过捕获标志。 */
    fan_tach_seeded = 0U;
    fan_tach_valid = 0U;
    fan_tach_period = 0U;
    fan_tach_suspended = 0U;
    if (fan_tach_irq_was_enabled != 0U)
    {
        __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1); /* 只恢复保存前已经使能的中断。 */
    }
    Fan_IO_Unlock(primask); /* 任务恢复后须等两个新边沿才得到完整有效周期。 */
}
