/**
  ******************************************************************************
  * @file        : proto.h
  * @brief       : 通用可重入串口帧解析器
  * @details     本模块提供通用的串口帧解析功能，支持三种帧格式：
  *              - 固定长度帧
  *              - 带长度字段的不定长帧
  *              - 以特定结尾标识的不定长帧
  *              特点：
  *              - 不做动态内存分配，所有缓冲区由用户提供
  *              - 支持可配置的CRC校验
  * @attention   使用前需要先调用 proto_init() 进行初始化
  ******************************************************************************
  */

#ifndef PROTO_H
#define PROTO_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include "kfifo.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 帧类型枚举
 */
typedef enum {
    FRAME_TYPE_FIXED = 0,       /**< 固定长度帧 */
    FRAME_TYPE_LEN_FIELD,       /**< 带长度字段的帧 */
    FRAME_TYPE_TAIL             /**< 以特定结尾标识的帧 */
} frame_type_t;

/**
 * @brief 解析状态枚举
 */
typedef enum {
    PROTO_STATE_FIND_HEAD = 0,  /**< 寻找帧头 */
    PROTO_STATE_READ_LEN,       /**< 读取长度字段 */
    PROTO_STATE_RECV_BODY,      /**< 接收帧体 */
    PROTO_STATE_DONE            /**< 帧接收完成 */
} proto_state_t;

/**
 * @brief 错误码枚举
 */
typedef enum {
    PROTO_ERR_NONE = 0,         /**< 无错误 */
    PROTO_ERR_HEAD,             /**< 帧头不匹配 */
    PROTO_ERR_LEN_FIELD,        /**< 长度字段超限（过大或过小） */
    PROTO_ERR_CHECKSUM,         /**< 校验错误 */
    PROTO_ERR_TAIL,             /**< 帧尾不匹配 */
    PROTO_ERR_BUF_TOO_SMALL,    /**< 帧接收缓冲区过小 */
    PROTO_ERR_TIMEOUT           /**< 接收超时 */
} proto_err_t;

/**
 * @brief 长度语义回调函数类型
 * @param frame_so_far 当前已接收的帧数据（包含帧头）
 * @param raw_len 从长度字段直接读取的原始值
 * @return 最终整帧长度（包含帧头、帧体、CRC、帧尾等）
 * @details 返回值必须是"最终整帧长度"
 */
typedef uint16_t (*proto_len_semantic_cb_t)(const uint8_t *frame_so_far,
                                            uint16_t raw_len);

/**
 * @brief 校验和计算回调函数类型
 * @param buf 待校验的数据缓冲区
 * @param len 待校验的数据长度
 * @return 计算得到的校验和值（用于与帧内字段比较）
 */
typedef uint32_t (*proto_checksum_cb_t)(const uint8_t *buf, uint16_t len);

/**
 * @brief 获取时间戳回调函数类型
 * @return 当前系统时间戳（单位：ms）
 */
typedef uint32_t (*proto_get_tick_cb_t)(void);

/**
 * @brief 帧格式描述结构体
 */
typedef struct {
    frame_type_t  type;                    /**< 帧类型 */

    const uint8_t *head;                   /**< 帧头数据指针 */
    uint8_t        head_len;               /**< 帧头长度 */
    const uint8_t *tail;                   /**< 帧尾数据指针 */
    uint8_t        tail_len;               /**< 帧尾长度 */
    
    /* 仅 FRAME_TYPE_FIXED 使用 */
    uint16_t       fixed_frame_len;        /**< 固定长度帧的总长度 */
    
    /* 仅 FRAME_TYPE_LEN_FIELD 使用 */
    uint8_t        len_field_offset;       /**< 长度字段在帧中的偏移量 */
    uint8_t        len_field_size;         /**< 长度字段大小（1/2/4字节，仅FRAME_TYPE_LEN_FIELD使用） */
    proto_len_semantic_cb_t len_cb;        /**< 长度语义回调函数（可为NULL） */

    uint8_t        csum_offset;            /**< 校验和计算的起始偏移（从帧头=0算的绝对偏移） */
    uint8_t        csum_size;              /**< 校验和字段大小（1/2/4字节） */
    proto_checksum_cb_t csum_cb;           /**< 校验和计算函数（可为NULL） */

    uint8_t        endian;                 /**< 字节序：0=小端(LE)，1=大端(BE) */

    uint16_t       max_frame_len;          /**< 最大帧长度限制（安全限制） */
    uint16_t       min_frame_len;          /**< 最小帧长度限制（用于快速判断数据是否足够，0表示自动计算） */
    uint32_t       frame_timeout_ms;       /**< 单帧接收超时时间（毫秒），0表示不使用超时 */

    void          *user_arg;               /**< 用户自定义参数（传递给回调函数） */
} proto_frame_fmt_t;

/**
 * @brief 协议解析器实例结构体
 */
typedef struct proto {
    const proto_frame_fmt_t *fmt;          /**< 帧格式描述指针 */

    uint8_t     *rx_buf;                   /**< 接收缓冲区指针 */
    uint16_t     rx_buf_size;              /**< 接收缓冲区大小 */

    uint16_t     cur_len;                  /**< 当前已接收的字节数 */
    uint16_t     expected_len;             /**< 期望接收的总长度 */

    proto_state_t state;                   /**< 当前解析状态 */
    uint8_t       head_match_pos;          /**< 帧头匹配位置 */

    uint32_t     frame_start_tick;         /**< 上次收到数据的时间戳（毫秒），用于数据流中断超时检测 */
    proto_get_tick_cb_t get_tick;          /**< 获取时间戳回调函数（可为NULL） */
    
    uint16_t     fifo_skip_count;           /**< 已确认需要跳过的 FIFO 字节数（仅在完整帧接收成功后使用） */

    void (*on_frame)(struct proto *p, const uint8_t *frame, uint16_t len);  /**< 帧接收完成回调 */
    void (*on_error)(struct proto *p, int err);                             /**< 错误回调 */
} proto_t;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化协议解析器实例
 */
int proto_init(proto_t *inst,
               const proto_frame_fmt_t *fmt,
               uint8_t *rx_buf,
               uint16_t rx_buf_size,
               void (*on_frame)(proto_t *, const uint8_t *, uint16_t),
               void (*on_error)(proto_t *, int));

/**
 * @brief 设置获取时间戳回调函数（用于超时检测）
 */
int proto_set_get_tick(proto_t *inst, proto_get_tick_cb_t get_tick);

/**
 * @brief 从kfifo轮询数据并解析
 */
int proto_poll_from_kfifo(proto_t *inst, kfifo_t *fifo);

/**
 * @brief 获取当前解析状态
 */
proto_state_t proto_get_state(const proto_t *inst);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_H */

