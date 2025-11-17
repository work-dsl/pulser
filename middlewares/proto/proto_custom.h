/**
  ******************************************************************************
  * @file        : proto_custom.h
  * @brief       : 自定义协议定义
  * @details     本文件定义了自定义通信协议格式和相关接口。
  *              协议格式：
  *              - FA: 帧头(1字节)
  *              - LEN(2): 整帧长度(含头尾，2字节小端)
  *              - DEV(1): 设备地址(1字节)
  *              - CMD(1): 命令码(1字节)
  *              - MOD(1): 模块标识(1字节)
  *              - DATA(...): 数据载荷(可变长度)
  *              - CRC16(2): CRC校验(2字节小端，在尾前2字节)
  *              - 0x0D: 帧尾(1字节)
  * @attention   使用前需要先配置CUSTOM_FMT并初始化协议解析器
  ******************************************************************************
  */

#ifndef CUSTOM_PROTO_H
#define CUSTOM_PROTO_H

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
    ACK_OK = 0x00,                  /**< 操作执行成功 */

    ACK_ERR_UNKNOWN,                /**< 0x01 未知错误 */
    
    /* 帧解析错误响应 */
    ACK_ERR_HEAD,                   /**< 帧头不匹配 */
    ACK_ERR_LEN_FIELD,              /**< 长度字段超限（过大或过小） */
    ACK_ERR_CHECKSUM,               /**< 校验错误 */
    ACK_ERR_TAIL,                   /**< 帧尾不匹配 */
    ACK_ERR_BUF_TOO_SMALL,          /**< 帧接收缓冲区过小 */
    ACK_ERR_TIMEOUT,                /**< 接收超时 */
    
    /* 操作错误响应 */
    ACK_ERR_DEV_ADDR,               /**< 设备地址错误 */
    ACK_ERR_MODULE_ADDR,            /**< 模块地址错误 */
    ACK_ERR_DATA_INVALID_PARAM,     /**< 数据参数无效 */
    ACK_ERR_UNSUP_CMD,              /**< 不支持的命令 */
    ACK_ERR_BUSY,                   /**< 正忙 */
    ACK_ERR_OPERATE_ABNORMAL,       /**< 操作异常 */
    ACK_ERR_MODE_ABNORMAL,          /**< 模式异常 */
    ACK_ERR_OPERATE_INVALID,        /**< 操作无效 */
    ACK_ERR_MODULE_LOCK,            /**< 模块锁定 */
    ACK_ERR_SYSTEM_LOCK,            /**< 系统锁定 */
    
    /* 针对耗时类命令 */
    ACK_IN_PROGERESS = 0x80,        /**< 0x80 已接受，正在执行 */
    ACK_TASK_DONE = 0x81            /**< 0x81 执行完成 */
} ack_code_t;

/* Exported constants --------------------------------------------------------*/

#define PROTO_CUSTOM_FRAME_MIN_LEN      (0x09)  /**< 自定义协议最小帧长度 */
#define PROTO_CUSTOM_FRAME_MAX_LEN      (64)    /**< 自定义协议最大帧长度 */

/**
 * @defgroup 通用命令码定义
 * @{
 */
#define CMD_HAND_SHAKE                  (0x01U) /**< 上电握手 */
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
#define CMD_STATUS_UPLOAD               (0x20U) /**< 状态上传 */
#define CMD_FRAME_PARSER_ERR_ACK        (0x21U) /**< 帧解析错误应答 */
/**
 * @}
 */
 
/* Exported variables --------------------------------------------------------*/

/**
 * @brief 自定义协议帧格式描述
 */
extern const proto_frame_fmt_t CUSTOM_FMT;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief CRC16校验回调函数（用于协议解析器）
 * @param buf 待校验的数据缓冲区
 * @param len 待校验的数据长度
 * @return CRC16校验值（32位，低16位有效）
 */
static inline uint32_t custom_crc16_cb(const uint8_t *buf, uint16_t len)
{
    return (uint32_t)crc16_modbus(buf, len);
}

/**
 * @brief 长度字段语义回调函数（用于协议解析器）
 * @param frame_so_far 当前已接收的帧数据（包含帧头）
 * @param raw_len 从长度字段直接读取的原始值
 * @return 最终整帧长度（本协议中等于raw_len）
 * @details 本协议的长度字段表示的就是整帧长度（含头尾）
 */
static inline uint16_t custom_len_semantic_total(const uint8_t *frame_so_far,
                                                uint16_t raw_len)
{
    (void)frame_so_far;
    return raw_len;
}

int custom_build_frame(uint8_t *out, uint16_t out_size,
                     uint8_t dev, uint8_t cmd, uint8_t mod,
                     const uint8_t *data, uint16_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_PROTO_H */
