/**
  ******************************************************************************
  * @file        : proto_custom.h
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 2024-12-XX
  * @brief       : 自定义协议定义
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.定义自定义通信协议格式和相关接口
  *                2.协议格式：FA + LEN(2) + DEV + CMD + MOD + DATA + CRC16(2) + 0x0D
  *                3.支持命令帧和应答帧两种格式
  *                4.应答帧包含应答码字段
  ******************************************************************************
  */
#ifndef __PROTO_CUSTOM_H__
#define __PROTO_CUSTOM_H__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "proto.h"
#include "crc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief ACK响应码枚举
 */
typedef enum {
    ACK_OK                   = 0x00,    /**< 操作执行成功 */
    ACK_ERR_UNKNOWN          = 0x01,    /**< 未知错误 */
    
    /* 帧解析错误 */
    ACK_ERR_LEN_FIELD        = 0x02,    /**< 长度字段超限（过大或过小） */
    ACK_ERR_CHECKSUM         = 0x03,    /**< 校验错误 */
    ACK_ERR_TAIL             = 0x04,    /**< 帧尾不匹配 */
    ACK_ERR_BUF_TOO_SMALL    = 0x05,    /**< 帧接收缓冲区过小 */
    ACK_ERR_TIMEOUT          = 0x06,    /**< 接收超时 */
    
    /* 操作错误 */
    ACK_ERR_PRODUCT_ADDR     = 0x11,    /**< 产品地址错误 */
    ACK_ERR_MODULE_ADDR      = 0x12,    /**< 模块地址错误 */
    ACK_ERR_INVALID_PARAM    = 0x13,    /**< 数据参数无效 */
    ACK_ERR_UNSUP_CMD        = 0x14,    /**< 不支持的命令 */
    ACK_ERR_BUSY             = 0x15,    /**< 操作正忙 */
    ACK_ERR_OPERATE_ABNORMAL = 0x16,    /**< 操作异常 */
    ACK_ERR_MODE_ABNORMAL    = 0x17,    /**< 模式异常 */
    ACK_ERR_OPERATE_INVALID  = 0x18,    /**< 操作无效 */
    ACK_ERR_MODULE_LOCK      = 0x19,    /**< 模块锁定 */
    ACK_ERR_SYSTEM_LOCK      = 0x20,    /**< 系统锁定 */
    
    /* 针对操作耗时类命令 */
    ACK_IN_PROGERESS         = 0x80,    /**< 已接受，正在执行 */
} ack_code_t;

/* Exported constants --------------------------------------------------------*/

#define PROTO_CUSTOM_FRAME_MIN_LEN      (0x09U)  /**< 自定义协议最小帧长度 */
#define PROTO_CUSTOM_FRAME_MAX_LEN      (64U)    /**< 自定义协议最大帧长度 */

/**
 * @defgroup 通用命令码定义
 * @{
 */
/**
 * @brief 上位机 -> 下位机
 */
#define CMD_HAND_SHAKE                  (0x01U) /**< 握手 */
#define CMD_GET_SOFTWARE_VERSION        (0x02U) /**< 查询软件版本信息 */
#define CMD_SET_HARDWARE_VERSION        (0x03U) /**< 设置硬件版本号信息 */
#define CMD_GET_HARDWARE_VERSION        (0x04U) /**< 查询硬件版本信息 */
#define CMD_SET_SERIAL_NUMBER           (0x05U) /**< 设置序列号信息 */
#define CMD_GET_SERIAL_NUMBER           (0x06U) /**< 查询序列号信息 */
#define CMD_CTRL_SOFT_RESET             (0x07U) /**< 系统复位控制 */
#define CMD_CTRL_SELF_CHECK             (0x08U) /**< 系统自检控制 */
#define CMD_CTRL_LOW_POWER_MODE         (0x09U) /**< 低功耗控制 */
#define CMD_CTRL_IAP                    (0x0AU) /**< 启动在线升级 */
#define CMD_CTRL_UPLOAD_MODE            (0x0BU) /**< 控制上传模式 */
#define CMD_STATUS_UPLOAD               (0x0CU) /**< 状态上传 */

/**
 * @brief 下位机 -> 上位机
 */
#define CMD_FRAME_PARSER_ERR_ACK        (0x2FU) /**< 帧解析错误应答 */
/**
 * @}
 */
 
/* Exported variables --------------------------------------------------------*/

/**
 * @brief 自定义协议帧格式描述
 */
extern const proto_cfg_t CUSTOM_FMT;

/* Exported functions --------------------------------------------------------*/

static inline uint32_t custom_crc16_cb(const uint8_t *buf, uint16_t len)
{
    return (uint32_t)crc16_modbus(buf, len);
}

static inline uint16_t custom_len_semantic_total(const uint8_t *frame_so_far,
                                                uint16_t raw_len)
{
    (void)frame_so_far;
    return raw_len;
}

int custom_build_frame(uint8_t *out, uint16_t out_size,
                     uint8_t dev, uint8_t cmd, uint8_t mod,
                     const uint8_t *data, uint16_t data_len);
                    
int custom_build_response_frame(uint8_t *out, uint16_t out_size,
                               uint8_t dev, uint8_t cmd, uint8_t mod,
                               uint8_t ack,
                               const uint8_t *data, uint16_t data_len);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PROTO_CUSTOM_H__ */
