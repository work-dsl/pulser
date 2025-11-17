/**
  ******************************************************************************
  * @file        : proto_custom.c
  * @brief       : 自定义协议实现
  * @details     本文件实现了proto_custom.h中定义的自定义协议功能，
  *              包括协议格式配置和帧构建功能。
  * @attention   使用前需要先调用 proto_init() 初始化协议解析器
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "proto_custom.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

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
const proto_frame_fmt_t CUSTOM_FMT = {
    .type             = FRAME_TYPE_LEN_FIELD,
    .head             = CUSTOM_HEAD,
    .head_len         = 1U,
    .tail             = CUSTOM_TAIL,
    .tail_len         = 1U,

    .fixed_frame_len  = 0U,

    .len_field_offset = 1U,            /* FA后的2字节为长度字段 */
    .len_field_size   = 2U,
    .len_cb           = custom_len_semantic_total,

    .csum_offset      = 1U,            /* CRC计算从长度字段开始 */
    .csum_size        = 2U,           /* CRC在尾前2字节 */
    .csum_cb          = custom_crc16_cb,

    .endian           = 0U,           /* 统一字节序：小端(LE) */
    
    .min_frame_len    = PROTO_CUSTOM_FRAME_MIN_LEN,
    .max_frame_len    = PROTO_CUSTOM_FRAME_MAX_LEN,
    .frame_timeout_ms = 50U,        /* 单帧接收超时50ms */
    .user_arg         = NULL
};

/* Private function prototypes -----------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 构建自定义协议帧
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
    uint16_t need;
    uint16_t pos = 0U;
    uint16_t crc_val;

    if (out == NULL) {
        return -1;
    }

    /* 固定开销 = 1(FA) + 2(LEN) + 1(DEV) + 1(CMD) + 1(MOD) + 2(CRC) + 1(TAIL) = 9 */
    need = (uint16_t)(9U + data_len);
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
