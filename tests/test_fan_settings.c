/* 双页Flash日志的主机故障注入测试；不加入Keil目标固件。 */
#include "fan_settings.h" /* 使用与实机相同的设置模块接口。 */
#include <assert.h> /* 用断言检查掉电恢复和写入边界。 */
#include <stdio.h> /* 输出测试汇总。 */
#include <string.h> /* 复制故障测试所需的模拟Flash快照。 */

#define TEST_HALFWORDS FAN_SETTINGS_PAGE_BYTES /* 双页总字节数除2后的半字数量。 */

static uint16_t test_flash[TEST_HALFWORDS]; /* 两个1KB模拟Flash页。 */
static int32_t test_budget = -1; /* 允许成功执行的半字修改次数；-1表示不限。 */
static uint32_t test_programs; /* 实际尝试的编程次数，用于验证相同值不写入。 */
static uint32_t test_erases; /* 实际尝试的页擦除次数。 */
static uint8_t test_drop_write; /* 模拟后端返回成功但未实际完成写入。 */

/** 核对模块没有越界访问预留的两个数据页。 */
static uint32_t Test_Index(uint32_t address)
{
    assert(address >= FAN_SETTINGS_PAGE0_ADDR); /* 地址不得进入程序区。 */
    assert(address < FAN_SETTINGS_PAGE1_ADDR + FAN_SETTINGS_PAGE_BYTES); /* 地址不得超出正式64KB Flash。 */
    assert((address & 1U) == 0U); /* F103半字访问必须偶数对齐。 */
    return (address - FAN_SETTINGS_PAGE0_ADDR) / 2U;
}

/** 每次修改前检查剩余预算，模拟任意写入位置突然断电。 */
static uint8_t Test_AllowMutation(void)
{
    if (test_budget == 0)
    {
        return 0U;
    }
    if (test_budget > 0)
    {
        --test_budget;
    }
    return 1U;
}

/** 设置模块的主机读取后端。 */
uint16_t Fan_Settings_TestRead(uint32_t address)
{
    return test_flash[Test_Index(address)]; /* 同时验证地址始终位于数据区。 */
}

/** 模拟真实Flash的单向位编程、断电及读回异常。 */
uint8_t Fan_Settings_TestProgram(uint32_t address, uint16_t value)
{
    uint32_t index = Test_Index(address); /* 转换到模拟Flash半字索引。 */
    ++test_programs;
    if (Test_AllowMutation() == 0U) /* 本次半字编程开始前模拟掉电。 */
    {
        return 0U;
    }
    assert((test_flash[index] & value) == value); /* 不能把旧记录的0位直接编程回1。 */
    if (test_drop_write != 0U)
    {
        test_drop_write = 0U;
        return 1U;
    }
    test_flash[index] = value;
    return 1U;
}

/** 将整页擦除分为512个半字步骤，模拟新页擦除途中掉电。 */
uint8_t Fan_Settings_TestErase(uint32_t address)
{
    uint32_t first = Test_Index(address); /* 待擦页首半字索引。 */
    uint32_t index; /* 当前页内半字偏移。 */
    assert(address == FAN_SETTINGS_PAGE0_ADDR || address == FAN_SETTINGS_PAGE1_ADDR); /* 只能擦完整预留页。 */
    ++test_erases;
    for (index = 0U; index < FAN_SETTINGS_PAGE_BYTES / 2U; ++index)
    {
        if (Test_AllowMutation() == 0U) /* 允许擦除中途保留混合新旧数据。 */
        {
            return 0U;
        }
        test_flash[first + index] = 0xFFFFU;
    }
    return 1U;
}

/** 清空模拟Flash和故障设置，模拟首次使用芯片。 */
static void Test_Reset(void)
{
    uint32_t index; /* 模拟Flash半字索引。 */
    for (index = 0U; index < TEST_HALFWORDS; ++index)
    {
        test_flash[index] = 0xFFFFU;
    }
    test_budget = -1;
    test_programs = 0U;
    test_erases = 0U;
    test_drop_write = 0U;
}

/** 读取并检查目标，确保Load每次从当前Flash重新恢复状态。 */
static void Test_Expect(uint16_t expected)
{
    uint16_t rpm = 65535U; /* 故意使用非法初值，验证Load确实填写结果。 */
    assert(Fan_Settings_Load(&rpm) == 1U); /* 读取必须存在已提交记录。 */
    assert(rpm == expected); /* 恢复值必须与最后有效目标一致。 */
}

/** 用独立字节流CRC参考实现生成指定序号，测试32位日志序号回绕。 */
static uint16_t Test_RecordCrc(const uint16_t *words)
{
    uint16_t crc = 0xFFFFU; /* CRC16-CCITT初值。 */
    uint32_t byte; /* 固定12字节有效载荷的当前字节索引。 */
    uint32_t bit; /* 当前字节的位索引。 */
    for (byte = 0U; byte < 12U; ++byte)
    {
        crc ^= (uint16_t)((uint16_t)((words[byte / 2U] >> ((byte & 1U) * 8U)) & 0xFFU) << 8U);
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }
    return crc;
}

/** 验证空白、非法设置、正常追加、相同值去重和损坏回退。 */
static void Test_BasicAndCorruption(void)
{
    uint16_t rpm = 0U; /* 用于检查空白回退值。 */
    uint32_t programs; /* 相同值保存前的编程计数。 */
    Test_Reset(); /* 从全擦除态开始。 */
    assert(Fan_Settings_Load(&rpm) == 0U && rpm == 600U); /* 空白记录返回默认值但不自动写入。 */
    assert(Fan_Settings_Load(0) == 0U); /* 空指针不得触发Flash扫描或写入。 */
    assert(Fan_Settings_Save(1501U) == 0U); /* 拒绝超过上限。 */
    assert(Fan_Settings_Save(650U) == 0U); /* 拒绝非100 RPM档位。 */
    assert(test_programs == 0U && test_erases == 0U); /* 非法参数不接触Flash写接口。 */
    assert(Fan_Settings_Save(0U) == 1U); /* 接受0 RPM档位。 */
    Test_Expect(0U); /* 0是有效目标，不作为未初始化标志。 */
    programs = test_programs;
    assert(Fan_Settings_Save(0U) == 1U && test_programs == programs); /* 相同值不重复编程。 */
    assert(Fan_Settings_Save(600U) == 1U); /* 追加第二条设置。 */
    assert(Fan_Settings_Save(1500U) == 1U); /* 接受上限档位。 */
    Test_Expect(1500U); /* 返回最新已提交记录。 */
    test_flash[16U + 6U] ^= 1U;
    Test_Expect(600U); /* 最新记录CRC损坏时回退上一条。 */
    test_flash[8U + 7U] = 0xFFFFU;
    Test_Expect(0U); /* 缺少最后提交标志的记录一律忽略。 */
    test_drop_write = 1U;
    assert(Fan_Settings_Save(700U) == 0U); /* 后端虚报成功时必须被读回验证拦截。 */
    Test_Expect(0U); /* 失败后不能把未提交数据视为有效配置。 */
}

/** 在每个半字提交位置掉电，检查旧记录不丢、断写槽不原址重用。 */
static void Test_AppendPowerCuts(void)
{
    uint16_t snapshot[TEST_HALFWORDS]; /* 第一条有效记录的Flash快照。 */
    uint32_t cut; /* 追加写入8个半字中的掉电位置。 */
    Test_Reset(); /* 准备初始有效配置。 */
    assert(Fan_Settings_Save(600U) == 1U); /* 写入受保护的旧目标。 */
    memcpy(snapshot, test_flash, sizeof(snapshot)); /* 保存用于重复故障注入的相同初始状态。 */
    for (cut = 0U; cut < 8U; ++cut)
    {
        memcpy(test_flash, snapshot, sizeof(snapshot)); /* 每次从相同的旧配置启动。 */
        test_budget = (int32_t)cut;
        assert(Fan_Settings_Save(900U) == 0U); /* 新配置尚未提交时模拟掉电。 */
        Test_Expect(600U); /* 重启仍能找回最后已提交目标。 */
        test_budget = -1;
        assert(Fan_Settings_Save(900U) == 1U); /* 恢复供电后跳过断写槽重新追加。 */
        Test_Expect(900U); /* 重试提交成功后才能返回新目标。 */
    }
}

/** 在跨页擦除及新记录提交的每个位置掉电，验证唯一最新页始终保留。 */
static void Test_PageSwapPowerCuts(void)
{
    uint16_t snapshot[TEST_HALFWORDS]; /* 两页写满后的Flash快照。 */
    uint32_t index; /* 构造足够多的变更记录，完成首次换页。 */
    uint32_t cut; /* 第二次换页的512个擦除半字和8个写入半字故障位置。 */
    Test_Reset(); /* 从全擦除态建立日志。 */
    for (index = 0U; index < 128U; ++index)
    {
        assert(Fan_Settings_Save((index & 1U) != 0U ? 600U : 500U) == 1U); /* 连续变更，填满两页。 */
    }
    Test_Expect(600U); /* 最新值在第二页，第一页可安全被下一轮擦除。 */
    memcpy(snapshot, test_flash, sizeof(snapshot)); /* 固定新旧有效页共存的初始状态。 */
    for (cut = 0U; cut < FAN_SETTINGS_PAGE_BYTES / 2U + 8U; ++cut)
    {
        memcpy(test_flash, snapshot, sizeof(snapshot)); /* 每次保留相同的完整最新页。 */
        test_budget = (int32_t)cut;
        assert(Fan_Settings_Save(1500U) == 0U); /* 在页擦除或提交期间模拟断电。 */
        Test_Expect(600U); /* 无论新页损坏到哪一处，都恢复第二页旧目标。 */
        test_budget = -1;
        assert(Fan_Settings_Save(1500U) == 1U); /* 故障消失后可继续正常换页保存。 */
        Test_Expect(1500U); /* 成功提交后选取新页。 */
    }
}

/** 验证序号FFFFFFFF到0回绕后仍正确选中最新记录。 */
static void Test_SequenceWrap(void)
{
    Test_Reset(); /* 单独构造回绕边界。 */
    assert(Fan_Settings_Save(600U) == 1U); /* 生成格式正确的首条记录。 */
    test_flash[2U] = 0xFFFEU;
    test_flash[3U] = 0xFFFFU;
    test_flash[6U] = Test_RecordCrc(test_flash); /* 为测试种子同步CRC。 */
    assert(Fan_Settings_Save(700U) == 1U); /* 下一条序号为FFFFFFFF。 */
    Test_Expect(700U); /* 回绕前选择最新记录。 */
    assert(Fan_Settings_Save(800U) == 1U); /* 下一条序号自然回绕为0。 */
    Test_Expect(800U); /* 不使用普通整数大小比较序号。 */
    assert(Fan_Settings_Save(900U) == 1U); /* 回绕后继续递增。 */
    Test_Expect(900U); /* 回绕不影响后续追加和读取。 */
}

/** 主机测试入口，覆盖528个写入/换页掉电位置及其他损坏场景。 */
int main(void)
{
    Test_BasicAndCorruption(); /* 检查边界、损坏回退及相同值不写。 */
    Test_AppendPowerCuts(); /* 检查8处追加掉电。 */
    Test_PageSwapPowerCuts(); /* 检查520处擦页和提交掉电。 */
    Test_SequenceWrap(); /* 检查32位序号回绕。 */
    puts("fan_settings: boundary/corruption/dedup + 528 power cuts + sequence wrap passed"); /* 输出可重复的验证结果。 */
    return 0;
}
