#ifndef FAN_SETTINGS_H
#define FAN_SETTINGS_H

#include <stdint.h> /* 使用固定宽度的转速及Flash地址类型。 */

#ifdef __cplusplus
extern "C" {
#endif

#define FAN_SETTINGS_DEFAULT_RPM 600U /* 空白或损坏Flash记录的默认目标转速。 */
#define FAN_SETTINGS_PAGE0_ADDR 0x0800F800UL /* 正式64KB Flash末尾倒数第二个1KB数据页。 */
#define FAN_SETTINGS_PAGE1_ADDR 0x0800FC00UL /* 正式64KB Flash末尾最后一个1KB数据页。 */
#define FAN_SETTINGS_PAGE_BYTES 1024U /* STM32F103C8的物理Flash页大小。 */

/** 从双页日志读取最新有效目标；返回1表示有效，返回0时填写默认600 RPM。 */
uint8_t Fan_Settings_Load(uint16_t *rpm);

/** 追加保存0～1500且为100倍数的目标；返回1表示已有相同记录或本次写入成功。 */
uint8_t Fan_Settings_Save(uint16_t rpm);

#ifdef FAN_SETTINGS_HOST_TEST
/** 主机测试后端：读取指定模拟Flash半字，不参与目标固件构建。 */
uint16_t Fan_Settings_TestRead(uint32_t address);
/** 主机测试后端：模拟只能由1写0的半字编程及故障。 */
uint8_t Fan_Settings_TestProgram(uint32_t address, uint16_t value);
/** 主机测试后端：模拟整页擦除及中途断电。 */
uint8_t Fan_Settings_TestErase(uint32_t address);
#endif

#ifdef __cplusplus
}
#endif

#endif
