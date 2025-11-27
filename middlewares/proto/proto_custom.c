/**
  ******************************************************************************
  * @file        : proto_custom.c
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 2024-12-XX
  * @brief       : 自定义协议实现
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.实现自定义协议格式配置和帧构建功能
  *                2.支持命令帧构建（custom_build_frame）
  *                3.支持应答帧构建（custom_build_response_frame）
  *                4.应答帧包含应答码字段，位于模块地址之后
  *
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "proto_custom.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 协议帧头常量
 */
static const uint8_t CUSTOM_HEAD[] = { 0xFAU };

/**
 * @brief 协议帧尾常量
 */
static const uint8_t CUSTOM_TAIL[] = { 0x0DU };

/* Exported variables  -------------------------------------------------------*/

/**
 * @brief 自定义协议帧格式描述
 * @details 协议格式：FA(1) + LEN(2) + DEV(1) + CMD(1) + MOD(1) + DATA(...) + CRC16(2) + 0x0D(1)
 */
const proto_cfg_t CUSTOM_FMT = {
    .type             = FRAME_TYPE_LEN_FIELD,
    .head             = CUSTOM_HEAD,
    .head_len         = 1U,
    .tail             = CUSTOM_TAIL,
    .tail_len         = 1U,

    .max_frame_len    = PROTO_CUSTOM_FRAME_MAX_LEN,
    .min_frame_len    = PROTO_CUSTOM_FRAME_MIN_LEN,

    .len_cfg = {
        .len_offset   = 1U,            /* FA后的2字节为长度字段 */
        .len_size     = 2U,
        .len_cb       = custom_len_semantic_total
    },

    .csum_offset      = 1U,            /* CRC计算从长度字段开始 */
    .csum_size        = 2U,           /* CRC在尾前2字节 */
    .csum_cb          = custom_crc16_cb,
    .is_big_endian    = 0U,           /* 统一字节序：小端(LE) */
    
    .timeout_ms       = 50U           /* 单帧接收超时50ms */
};

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 内部辅助函数：构建协议帧的公共部分
 * @param out 输出缓冲区指针
 * @param out_size 输出缓冲区大小
 * @param dev 设备地址
 * @param cmd 命令码
 * @param mod 模块标识
 * @param ack 应答码（如果为0xFF表示无应答码字段）
 * @param data 数据载荷指针（可为NULL）
 * @param data_len 数据载荷长度
 * @param fixed_overhead 固定开销字节数（9或10）
 * @retval >=0 构建的帧长度（字节数）
 * @retval -1 失败（缓冲区太小或参数无效）
 * @details 统一处理命令帧和应答帧的构建逻辑
 */
static int custom_build_frame_internal(uint8_t *out, uint16_t out_size,
                                       uint8_t dev, uint8_t cmd, uint8_t mod,
                                       uint8_t ack,
                                       const uint8_t *data, uint16_t data_len,
                                       uint16_t fixed_overhead);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 构建自定义协议命令帧
 * @param out 输出缓冲区指针
 * @param out_size 输出缓冲区大小
 * @param dev 设备地址
 * @param cmd 命令码
 * @param mod 模块标识
 * @param data 数据载荷指针（可为NULL）
 * @param data_len 数据载荷长度
 * @retval >=0 构建的帧长度（字节数）
 * @retval -1 失败（缓冲区太小或参数无效）
 * @details 帧格式：FA + LEN(2) + DEV + CMD + MOD + DATA + CRC(2) + 0x0D
 *          固定开销为9字节（不含DATA部分）
 */
int custom_build_frame(uint8_t *out, uint16_t out_size,
                     uint8_t dev, uint8_t cmd, uint8_t mod,
                     const uint8_t *data, uint16_t data_len)
{
    /* 命令帧无应答码字段，使用0xFF作为标记 */
    return custom_build_frame_internal(out, out_size, dev, cmd, mod,
                                       0xFFU, data, data_len, 9U);
}

/**
 * @brief 构建自定义协议应答帧
 * @param out 输出缓冲区指针
 * @param out_size 输出缓冲区大小
 * @param dev 设备地址
 * @param cmd 命令码
 * @param mod 模块标识
 * @param ack 应答码
 * @param data 数据载荷指针（可为NULL）
 * @param data_len 数据载荷长度
 * @retval >=0 构建的帧长度（字节数）
 * @retval -1 失败（缓冲区太小或参数无效）
 * @details 应答帧格式：FA + LEN(2) + DEV + CMD + MOD + ACK + DATA + CRC(2) + 0x0D
 *          固定开销为10字节（不含DATA部分）
 *          应答码字段位于模块地址之后，数据载荷之前
 */
int custom_build_response_frame(uint8_t *out, uint16_t out_size,
                               uint8_t dev, uint8_t cmd, uint8_t mod,
                               uint8_t ack,
                               const uint8_t *data, uint16_t data_len)
{
    /* 应答帧包含应答码字段，固定开销为10字节 */
    return custom_build_frame_internal(out, out_size, dev, cmd, mod,
                                       ack, data, data_len, 10U);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 内部辅助函数：构建协议帧的公共部分
 * @param out 输出缓冲区指针
 * @param out_size 输出缓冲区大小
 * @param dev 设备地址
 * @param cmd 命令码
 * @param mod 模块标识
 * @param ack 应答码（如果为0xFF表示无应答码字段）
 * @param data 数据载荷指针（可为NULL）
 * @param data_len 数据载荷长度
 * @param fixed_overhead 固定开销字节数（9或10）
 * @retval >=0 构建的帧长度（字节数）
 * @retval -1 失败（缓冲区太小或参数无效）
 * @details 统一处理命令帧和应答帧的构建逻辑
 */
static int custom_build_frame_internal(uint8_t *out, uint16_t out_size,
                                       uint8_t dev, uint8_t cmd, uint8_t mod,
                                       uint8_t ack,
                                       const uint8_t *data, uint16_t data_len,
                                       uint16_t fixed_overhead)
{
    uint16_t need;
    uint16_t pos = 0U;
    uint16_t crc_val;
    uint8_t has_ack;

    if (out == NULL) {
        return -1;
    }

    /* 判断是否有应答码字段 */
    has_ack = (ack != 0xFFU) ? 1U : 0U;

    /* 计算总长度 */
    need = (uint16_t)(fixed_overhead + data_len);
    if (out_size < need) {
        return -1;
    }

    /* 帧头 */
    out[pos] = 0xFAU;
    pos++;

    /* 填长度（小端） */
    out[pos] = (uint8_t)(need & 0xFFU);
    pos++;
    out[pos] = (uint8_t)(need >> 8U);
    pos++;

    /* 设备地址、命令码、模块标识 */
    out[pos] = dev;
    pos++;
    out[pos] = cmd;
    pos++;
    out[pos] = mod;
    pos++;

    /* 应答码（仅应答帧包含） */
    if (has_ack != 0U) {
        out[pos] = ack;
        pos++;
    }

    /* 数据载荷 */
    if ((data_len > 0U) && (data != NULL)) {
        (void)memcpy(&out[pos], data, data_len);
        pos = (uint16_t)(pos + data_len);
    }

    /* CRC计算从长度字段开始（csum_offset=1），不包含帧头FA */
    /* 计算范围：从out[1]（长度字段）到out[pos-1]（数据载荷结束） */
    crc_val = crc16_modbus(&out[1U], (size_t)(pos - 1U));
    out[pos] = (uint8_t)(crc_val & 0xFFU);
    pos++;
    out[pos] = (uint8_t)(crc_val >> 8U);
    pos++;

    /* 帧尾 */
    out[pos] = 0x0DU;
    pos++;

    return (int)need;
}