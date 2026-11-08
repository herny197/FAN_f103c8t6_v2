#include "fan_settings.h" /* 公开接口和独立Flash数据区边界。 */
#ifndef FAN_SETTINGS_HOST_TEST
#include "main.h" /* 使用本工程HAL Flash编程接口；不改动生成驱动。 */
#endif

#define SETTINGS_WORDS 8U /* 单条日志为8个半字，即16字节，每页64条。 */
#define SETTINGS_RECORD_BYTES (SETTINGS_WORDS * 2U) /* 固定记录跨度，不依赖结构体对齐。 */
#define SETTINGS_MAGIC 0x46A1U /* 当前风扇配置记录类型。 */
#define SETTINGS_VERSION 1U /* 记录格式版本；旧格式不作有效数据使用。 */
#define SETTINGS_COMMIT 0xA55AU /* 最后一个半字写入的提交标记。 */

typedef struct
{
    uint32_t address; /* 最新有效记录所在物理地址；0表示没有有效记录。 */
    uint32_t sequence; /* 最新有效记录序号，使用模2^32比较支持回绕。 */
    uint16_t rpm; /* 经过范围和校验检查的目标转速。 */
} SettingsLatest;

/** 读取真实或主机模拟Flash半字，地址始终为数据区内偶数地址。 */
static uint16_t Settings_Read(uint32_t address)
{
#ifdef FAN_SETTINGS_HOST_TEST
    return Fan_Settings_TestRead(address); /* 调用故障可注入的主机读取接口。 */
#else
    return *(volatile const uint16_t *)address; /* 只读访问已预留的内部Flash。 */
#endif
}

/** 对前六个半字的低字节、高字节依次执行CRC16-CCITT，初值0xFFFF。 */
static uint16_t Settings_Crc(const uint16_t *words)
{
    uint16_t crc = 0xFFFFU; /* CRC16累积值。 */
    uint16_t byte_value; /* 当前按小端格式送入校验的字节。 */
    uint32_t index; /* 前六个半字索引。 */
    uint32_t part; /* 半字内的低/高字节索引。 */
    uint32_t bit; /* 当前字节的位迭代索引。 */

    for (index = 0U; index < 6U; ++index)
    {
        for (part = 0U; part < 2U; ++part)
        {
            byte_value = (uint16_t)((words[index] >> (part * 8U)) & 0xFFU);
            crc = (uint16_t)(crc ^ (uint16_t)(byte_value << 8U));
            for (bit = 0U; bit < 8U; ++bit)
            {
                crc = (uint16_t)((crc & 0x8000U) != 0U ?
                    (uint16_t)((uint16_t)(crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U));
            }
        }
    }
    return crc;
}

/** 目标仅接受用户确认的0～1500 RPM、每100 RPM一档。 */
static uint8_t Settings_RpmValid(uint16_t rpm)
{
    return (uint8_t)((rpm <= 1500U) && ((rpm % 100U) == 0U));
}

/** 读取固定16字节记录，并拒绝断电半写、错误版本和损坏数据。 */
static uint8_t Settings_ReadRecord(uint32_t address, uint16_t *words)
{
    uint32_t index; /* 当前记录内半字索引。 */

    for (index = 0U; index < SETTINGS_WORDS; ++index)
    {
        words[index] = Settings_Read(address + index * 2U); /* 读取全部半字，含最后提交标志。 */
    }
    return (uint8_t)((words[0] == SETTINGS_MAGIC) && (words[1] == SETTINGS_VERSION) &&
        (words[7] == SETTINGS_COMMIT) && Settings_RpmValid(words[4]) && /* 校验目标范围。 */
        ((uint16_t)(words[4] ^ words[5]) == 0xFFFFU) &&
        (words[6] == Settings_Crc(words))); /* 提交标志和CRC必须同时有效。 */
}

/** 模序号比较；相差少于2^31条记录时支持UINT32_MAX回绕。 */
static uint8_t Settings_IsNewer(uint32_t candidate, uint32_t previous)
{
    uint32_t distance = candidate - previous; /* 无符号减法按2^32自然回绕。 */
    return (uint8_t)((distance != 0U) && (distance < 0x80000000UL));
}

/** 扫描两页所有已提交记录；不通过页头状态推断有效配置。 */
static SettingsLatest Settings_Scan(void)
{
    SettingsLatest latest = {0U, 0U, FAN_SETTINGS_DEFAULT_RPM}; /* 没有有效记录时的默认状态。 */
    uint16_t words[SETTINGS_WORDS]; /* 当前固定大小记录缓冲。 */
    uint32_t page; /* 两个数据页的索引。 */
    uint32_t offset; /* 当前页内记录偏移。 */
    uint32_t address; /* 当前记录的实际Flash地址。 */
    uint32_t sequence; /* 当前记录的32位序号。 */

    for (page = 0U; page < 2U; ++page)
    {
        for (offset = 0U; offset < FAN_SETTINGS_PAGE_BYTES; offset += SETTINGS_RECORD_BYTES)
        {
            address = FAN_SETTINGS_PAGE0_ADDR + page * FAN_SETTINGS_PAGE_BYTES + offset;
            if (Settings_ReadRecord(address, words) != 0U) /* 校验每条候选配置。 */
            {
                sequence = (uint32_t)words[2] | ((uint32_t)words[3] << 16U);
                if ((latest.address == 0U) || Settings_IsNewer(sequence, latest.sequence)) /* 挑选最新有效记录。 */
                {
                    latest.address = address;
                    latest.sequence = sequence;
                    latest.rpm = words[4];
                }
            }
        }
    }
    return latest;
}

/** 查找全16字节都为空的槽位，损坏或断电未完成的槽位永不原址重写。 */
static uint32_t Settings_FindEmpty(uint32_t page_address)
{
    uint32_t offset; /* 当前页的记录偏移。 */
    uint32_t index; /* 当前记录内半字偏移。 */
    uint8_t empty; /* 当前记录是否全部为擦除态0xFFFF。 */

    for (offset = 0U; offset < FAN_SETTINGS_PAGE_BYTES; offset += SETTINGS_RECORD_BYTES)
    {
        empty = 1U;
        for (index = 0U; index < SETTINGS_WORDS; ++index)
        {
            if (Settings_Read(page_address + offset + index * 2U) != 0xFFFFU) /* 核对整个槽位。 */
            {
                empty = 0U;
                break;
            }
        }
        if (empty != 0U)
        {
            return page_address + offset;
        }
    }
    return 0U;
}

/** 编程一个半字，并立刻检查读回值；不关闭全局中断。 */
static uint8_t Settings_Program(uint32_t address, uint16_t value)
{
#ifdef FAN_SETTINGS_HOST_TEST
    if (Fan_Settings_TestProgram(address, value) == 0U) /* 主机后端可在任意半字前模拟掉电。 */
#else
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, value) != HAL_OK) /* 使用F103半字写入。 */
#endif
    {
        return 0U;
    }
    return (uint8_t)(Settings_Read(address) == value); /* 读回失败不提交配置。 */
}

/** 仅擦除需要接管的新页；调用方始终保留最新有效记录所在旧页。 */
static uint8_t Settings_Erase(uint32_t address)
{
#ifdef FAN_SETTINGS_HOST_TEST
    return Fan_Settings_TestErase(address); /* 模拟可中断的整页擦除。 */
#else
    FLASH_EraseInitTypeDef erase = {0}; /* 本次唯一数据页擦除参数。 */
    uint32_t page_error = 0U; /* HAL返回的失败页地址。 */
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = address;
    erase.NbPages = 1U;
    return (uint8_t)(HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK); /* 只擦一页，不使用全片擦除。 */
#endif
}

/** 获取最新设置；调用者在user_Init中使用返回值区分恢复与默认。 */
uint8_t Fan_Settings_Load(uint16_t *rpm)
{
    SettingsLatest latest; /* 扫描所得最新有效日志。 */
    if (rpm == 0)
    {
        return 0U;
    }
    latest = Settings_Scan(); /* 启动只读扫描，不自动擦除或写入默认值。 */
    *rpm = latest.rpm;
    return (uint8_t)(latest.address != 0U);
}

/** 追加保存目标；成功提交新记录之前绝不擦除唯一的最新有效页。 */
uint8_t Fan_Settings_Save(uint16_t rpm)
{
    SettingsLatest latest; /* 当前最新有效配置，用于相同值去重及安全换页。 */
    uint32_t active_page; /* 最新有效记录所在页，空白时优先第一页。 */
    uint32_t address; /* 本次追加记录的目标槽位。 */
    uint32_t sequence; /* 本次配置的递增序号。 */
    uint32_t index; /* 本次编程的半字索引。 */
    uint16_t words[SETTINGS_WORDS]; /* 本次16字节完整记录。 */
    uint16_t verify[SETTINGS_WORDS]; /* 最终提交后的读回校验缓冲。 */
    uint8_t success = 1U; /* 任一步写入失败均返回0，由唯一写任务保留待保存标志。 */

    if (Settings_RpmValid(rpm) == 0U) /* 拒绝范围外及非100倍数设置。 */
    {
        return 0U;
    }
    latest = Settings_Scan(); /* 每次重新扫描，失败重试不依赖易失缓存。 */
    if ((latest.address != 0U) && (latest.rpm == rpm))
    {
        return 1U;
    }
    active_page = (latest.address >= FAN_SETTINGS_PAGE1_ADDR) ? FAN_SETTINGS_PAGE1_ADDR : FAN_SETTINGS_PAGE0_ADDR;
    address = Settings_FindEmpty(active_page); /* 跳过损坏槽，寻找可直接追加的空槽。 */
#ifndef FAN_SETTINGS_HOST_TEST
    if (HAL_FLASH_Unlock() != HAL_OK) /* 只在确有待写变更时解锁Flash。 */
    {
        return 0U;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); /* 清除此前操作标志。 */
#endif
    if (address == 0U)
    {
        active_page = (active_page == FAN_SETTINGS_PAGE0_ADDR) ? FAN_SETTINGS_PAGE1_ADDR : FAN_SETTINGS_PAGE0_ADDR;
        success = Settings_Erase(active_page); /* 旧有效页保留，新页擦除失败时仍能恢复原值。 */
        address = active_page;
    }
    sequence = latest.sequence + 1U;
    words[0] = SETTINGS_MAGIC;
    words[1] = SETTINGS_VERSION;
    words[2] = (uint16_t)sequence;
    words[3] = (uint16_t)(sequence >> 16U);
    words[4] = rpm;
    words[5] = (uint16_t)~rpm;
    words[6] = Settings_Crc(words); /* 对类型、版本、序号、目标及反码一起校验。 */
    words[7] = SETTINGS_COMMIT;
    for (index = 0U; (index < SETTINGS_WORDS) && (success != 0U); ++index)
    {
        success = Settings_Program(address + index * 2U, words[index]); /* 提交标志为最后一次半字写入。 */
    }
    if (success != 0U)
    {
        success = Settings_ReadRecord(address, verify); /* 已提交数据必须再次通过完整CRC校验。 */
    }
#ifndef FAN_SETTINGS_HOST_TEST
    (void)HAL_FLASH_Lock(); /* 无论写入成败都重新锁定Flash。 */
#endif
    return success;
}
