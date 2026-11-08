#ifndef USER_WORK_H
#define USER_WORK_H

#include "fan_control.h" /* 业务控制状态和固定宽度类型。 */

#ifdef __cplusplus
extern "C" {
#endif

/* 控制任务发布的运行快照，其他任务必须通过一致复制读取。 */
typedef struct
{
    Fan_Control_t control; /* 目标、使能、占空比和状态机快照。 */
    uint32_t measured_rpm; /* 最近测得的RPM，无效时为0。 */
    uint32_t input_overflows; /* 输入事件队列异常满次数。 */
    uint32_t input_epoch; /* 启停代次，用于拒绝暂停期间的旧旋转消息。 */
    uint16_t saved_rpm; /* 最近成功保存的目标。 */
    uint8_t tach_valid; /* 当前是否有有效实际转速。 */
    uint8_t hardware_ok; /* 定时器业务是否启动成功。 */
    uint8_t saved_valid; /* Flash是否存在有效记录。 */
    uint8_t storage_error; /* 上次保存失败标志。 */
} Fan_AppStatus_t;

/* 调试观察用状态，禁止其他模块直接改写其中的控制字段。 */
extern volatile Fan_AppStatus_t g_fan_status;
/* 默认任务常驻循环之前调用一次，集中恢复设置并初始化业务。 */
void user_Init(void);

/**
 * @brief 默认任务 Task1 的用户业务入口，内部管理常驻循环及等待。
 */
void My_Task1(void);

/**
 * @brief 第二个任务 Task2 的用户业务入口，内部管理常驻循环及等待。
 */
void My_Task2(void);

/**
 * @brief 第三个任务 Task3 的用户业务入口，内部管理常驻循环及等待。
 */
void My_Task3(void);

#ifdef __cplusplus
}
#endif

#endif /* USER_WORK_H */
