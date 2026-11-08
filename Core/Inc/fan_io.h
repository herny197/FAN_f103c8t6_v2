#ifndef FAN_IO_H
#define FAN_IO_H

#include <stdint.h> /* 使用固定宽度的输入、测速及占空比类型。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 编码器方向按实板调整；每格计数与风扇每转脉冲数仍需实测确认。 */
#define FAN_ENCODER_COUNTS_PER_DETENT 4
#define FAN_ENCODER_DIRECTION        (-1) /* 实测顺时针升速，逆时针降速。 */
#define FAN_TACH_PULSES_PER_REV      2U
#define FAN_TACH_TIMEOUT_MS          1000U

/* 由输入任务提交的一次旋转/按键事件；停止期间不产生旋转步数。 */
typedef struct
{
    int16_t steps;             /* 本次机械格数，正数增加目标转速。 */
    uint8_t button_pressed;    /* 去抖后新按下事件，一次按压只置位一次。 */
    uint8_t rotated;           /* 运行时出现原始计数变化，用于重置停止旋转保存计时。 */
} Fan_Input_t;

/* 控制任务取得的完整测速快照；无有效反馈时rpm为0且valid为0。 */
typedef struct
{
    uint32_t rpm;              /* 按当前每转脉冲数计算的实际转速。 */
    uint32_t sequence;         /* 有效捕获周期序号，用于拒绝重复旧反馈。 */
    uint8_t valid;             /* 捕获有效且未超过无反馈超时。 */
} Fan_Tach_t;

/* 在安全0%风扇请求下启动三个定时器业务；成功返回1，失败返回0。 */
uint8_t Fan_IO_Init(void);
/* 每约5ms采集硬件编码器计数和按键；暂停时也持续消耗旧计数。 */
void Fan_IO_PollInput(uint32_t now_ms, uint8_t allow_rotation, Fan_Input_t *input);
/* 设置风扇输入端千分比占空比，内部处理板上单级NPN反相。 */
void Fan_IO_SetDuty(uint16_t permille);
/* 按LED5到LED1累计点亮0到5灯，低电平有效。 */
void Fan_IO_SetLeds(uint8_t count);
/* 取得一致测速快照；计算放在任务中，捕获中断仅记录计时数据。 */
void Fan_IO_GetTach(uint32_t now_ms, Fan_Tach_t *out);
/* Flash擦写前屏蔽TIM3捕获事件并使旧反馈失效，硬件PWM继续输出。 */
void Fan_IO_FlashBegin(void);
/* Flash擦写后恢复捕获并丢弃首个捕获周期，避免跨擦写样本误判。 */
void Fan_IO_FlashEnd(void);

#ifdef __cplusplus
}
#endif

#endif
