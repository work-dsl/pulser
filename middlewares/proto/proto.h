/**
  ******************************************************************************
  * @file        : proto.h
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 20xx-xx-xx
  * @brief       : 通用协议解析器接口定义
  * @details     本文件定义了通用协议解析器的接口和数据结构。
  *              职责：
  *              - 定义协议解析器的配置结构体
  *              - 定义协议解析器的运行时状态结构体
  *              - 定义协议解析器的API接口
  *              - 支持固定长度、长度字段、尾部匹配三种帧类型
  * @attention   使用前需要先调用 proto_init() 进行初始化
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.定义通用协议解析器接口
  *                2.支持多种帧类型（固定长度、长度字段、尾部匹配）
  *                3.支持校验和验证和超时检测
  *
  ******************************************************************************
  */
#ifndef PROTO_H
#define PROTO_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "kfifo.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 帧类型枚举
 */
typedef enum {
    FRAME_TYPE_FIXED = 0,       /**< 固定长度帧（长度由 max_frame_len 指定） */
    FRAME_TYPE_LEN_FIELD,       /**< 带长度字段的帧 */
    FRAME_TYPE_TAIL             /**< 以特定结尾结束的帧（不定长） */
} frame_type_t;

/**
 * @brief 协议解析错误码枚举
 */
typedef enum {
    PROTO_ERR_NONE = 0,         /**< 无错误 */
    PROTO_ERR_LEN_INVALID,      /**< 长度非法（超过 max_frame_len 或 FIFO 容量） */
    PROTO_ERR_CHECKSUM,         /**< 校验和失败 */
    PROTO_ERR_TAIL,             /**< 尾部匹配失败 */
    PROTO_ERR_OUT_FIFO_FULL,    /**< 输出队列已满，有效载荷丢失 */
    PROTO_ERR_BUFF_OVERFLOW,    /**< 缓冲区溢出（仅针对 FRAME_TYPE_TAIL） */
    PROTO_ERR_TIMEOUT           /**< 接收帧超时 */
} proto_err_t;

/**
 * @brief 长度字段解码回调函数类型
 * @param frame_head 帧头数据指针
 * @param raw_len 从长度字段直接读取的原始值
 * @return 最终整帧长度
 * @details 用于对长度字段进行语义转换，例如长度字段可能表示payload长度，
 *          需要加上帧头、帧尾、校验等固定开销才是整帧长度
 */
typedef uint16_t (*proto_len_dec_cb_t)(const uint8_t *frame_head, uint16_t raw_len);

/**
 * @brief 校验和计算回调函数类型
 * @param data 待校验的数据缓冲区
 * @param len 待校验的数据长度
 * @return 校验和值（32位，实际使用的位数由 csum_size 决定）
 */
typedef uint32_t (*proto_checksum_cb_t)(const uint8_t *data, uint16_t len);

/**
 * @brief 时间戳获取回调函数类型
 * @return 当前时间戳（毫秒）
 * @details 用于超时检测，需要返回单调递增的时间戳
 */
typedef uint32_t (*proto_tick_cb_t)(void);

/**
 * @brief 错误回调函数类型
 * @param proto_inst 协议解析器实例指针（void *类型，需转换为proto_t *）
 * @param err 错误码
 * @details 当协议解析器检测到错误时调用此函数
 */
typedef void (*proto_error_cb_t)(void *proto_inst, proto_err_t err);

/**
 * @brief 长度字段配置结构体
 * @details 仅当帧类型为 FRAME_TYPE_LEN_FIELD 时有效
 */
typedef struct {
    uint8_t             len_offset;     /**< 长度字段相对于帧头起始字节的偏移 */
    uint8_t             len_size;       /**< 长度字段占用的字节数（1、2、4） */
    proto_len_dec_cb_t  len_cb;         /**< 可选：长度数值转换回调（可为NULL） */
} proto_len_cfg_t;

/**
 * @brief 协议配置结构体
 * @details 建议定义为 const 类型，在初始化时传入
 */
typedef struct {
    frame_type_t        type;           /**< 帧类型 */
    
    /* 帧头与帧尾定义 */
    const uint8_t      *head;           /**< 帧头序列指针 */
    uint8_t             head_len;       /**< 帧头长度 */
    const uint8_t      *tail;           /**< 帧尾序列指针（可为NULL） */
    uint8_t             tail_len;       /**< 帧尾长度（无尾则填0） */
    
    /* 长度限制 */
    uint16_t            max_frame_len;   /**< 最大允许帧长（对于 FIXED 模式，此值即为固定长度） */
    uint16_t            min_frame_len;   /**< 最小允许帧长（若为0，代码内不进行最小检查） */
    
    /* 长度字段参数（仅 FRAME_TYPE_LEN_FIELD 有效） */
    proto_len_cfg_t     len_cfg;
    
    /* 校验参数 */
    uint8_t             csum_offset;    /**< 校验计算起始偏移（相对于帧头） */
    uint8_t             csum_size;      /**< 校验值长度（0、1、2、4字节） */
    proto_checksum_cb_t csum_cb;         /**< 校验计算回调（可为NULL） */
    uint8_t             is_big_endian;  /**< 字节序：1=大端，0=小端（用于读取长度和校验值） */
    
    uint32_t            timeout_ms;      /**< 帧内超时时间（0表示不启用超时检测） */
} proto_cfg_t;

/**
 * @brief 协议解析器状态枚举
 */
typedef enum {
    P_STATE_FIND_HEAD = 0,      /**< 寻找帧头状态 */
    P_STATE_READ_LEN,           /**< 读取长度字段状态 */
    P_STATE_RECV_BODY,          /**< 接收帧体状态 */
    P_STATE_MATCH_TAIL          /**< 匹配尾部状态 */
} p_state_t;

/**
 * @brief 协议解析器运行时控制块
 */
typedef struct {
    const proto_cfg_t  *cfg;            /**< 协议配置指针 */
    
    /* 缓冲区与队列 */
    uint8_t            *rx_buf;         /**< 线性缓冲区，用于组包 */
    uint16_t            buf_size;       /**< rx_buf 的大小 */
    kfifo_t            *out_fifo;       /**< 解析完成的有效数据推入此队列 */
    
    /* 内部状态 */
    p_state_t           state;          /**< 当前解析状态 */
    uint16_t            w_idx;          /**< 当前 rx_buf 写入索引 */
    uint16_t            target_len;     /**< 当前帧的目标长度 */
    uint32_t            last_tick;       /**< 上次接收字节的时间戳 */
    
    /* 接口回调 */
    proto_tick_cb_t     get_tick;       /**< 时间戳获取回调（可为NULL） */
    proto_error_cb_t    on_error;       /**< 错误回调（可为NULL） */
} proto_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

int  proto_init         (proto_t *inst, const proto_cfg_t *cfg, 
                        uint8_t *rx_buf, uint16_t buf_size,
                        kfifo_t *out_fifo, proto_error_cb_t on_err);
void proto_set_tick_cb  (proto_t *inst, proto_tick_cb_t get_tick);
int  proto_poll         (proto_t *inst, kfifo_t *in_fifo);
void proto_reset        (proto_t *inst);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PROTO_H */
