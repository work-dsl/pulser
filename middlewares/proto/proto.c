/**
  ******************************************************************************
  * @file        : proto.c
  * @brief       : 通用可重入串口帧解析器实现
  * @details     本文件实现了proto.h中定义的协议解析器功能。
  * @attention   使用前需要先调用 proto_init() 进行初始化
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "proto.h"
#include <string.h>
#include "byteorder.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static inline uint32_t read_u32(const uint8_t *p, uint8_t size, uint8_t be);
static inline void proto_reset(proto_t *p);
static inline uint8_t proto_tail_cmp(const uint8_t *buf, const uint8_t *tail, uint8_t len);
static inline void proto_update_tick(proto_t *p);
static inline void proto_report_error(proto_t *p, int err);
static inline uint8_t proto_check_buf_overflow(proto_t *p);
static inline uint16_t proto_get_fifo_offset(const proto_t *p);
static uint8_t proto_try_match_head_fast(proto_t *inst,
                                         kfifo_t *fifo,
                                         unsigned int fifo_len,
                                         unsigned int *skip_bytes);
static uint8_t proto_peek_fifo_bytes(kfifo_t *fifo,
                                     unsigned int offset,
                                     uint8_t *dst,
                                     unsigned int len);
static uint8_t proto_append_from_fifo(proto_t *inst,
                                      kfifo_t *fifo,
                                      unsigned int fifo_len,
                                      uint16_t bytes_needed);
static void proto_finish_frame(proto_t *p);
static void proto_feed_byte(proto_t *p, uint8_t byte);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 读取整数（小工具函数，优化版）
 * @param p 数据指针
 * @param size 数据大小（1/2/4字节）
 * @param be 字节序：0=小端(LE)，1=大端(BE)
 * @return 读取的整数值（转换为CPU字节序）
 * @details 使用byteorder.h中的函数进行大小端转换，优化性能
 *          对于小端数据，CPU本身就是小端，可直接读取
 */
static inline uint32_t read_u32(const uint8_t *p, uint8_t size, uint8_t be)
{
    uint32_t v;
    
    /* 使用 switch 优化分支预测，size 通常为编译时常量 */
    switch (size) {
    case 1U:
        v = (uint32_t)p[0];
        break;
    case 2U: {
        uint16_t v16;
        if (be != 0U) {
            /* 大端数据：p[0]是最高字节，p[1]是最低字节 */
            v16 = ((uint16_t)p[0] << 8U) | (uint16_t)p[1];
            /* 转换为小端（CPU端） */
            v = (uint32_t)be16_to_cpu(v16);
        } else {
            /* 小端数据：CPU本身就是小端，直接读取 */
            v = (uint32_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8U));
        }
        break;
    }
    case 4U: {
        uint32_t v32;
        if (be != 0U) {
            /* 大端数据：p[0]是最高字节，p[3]是最低字节 */
            v32 = ((uint32_t)p[0] << 24U) | ((uint32_t)p[1] << 16U) |
                  ((uint32_t)p[2] << 8U)  | (uint32_t)p[3];
            /* 转换为小端（CPU端） */
            v = be32_to_cpu(v32);
        } else {
            /* 小端数据：CPU本身就是小端，直接读取 */
            v = ((uint32_t)p[0]) | ((uint32_t)p[1] << 8U) |
                ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
        }
        break;
    }
    default:
        v = 0U;
        break;
    }
    return v;
}

/**
 * @brief 重置协议解析器状态
 * @param p 协议解析器实例指针
 */
static inline void proto_reset(proto_t *p)
{
    p->state = PROTO_STATE_FIND_HEAD;
    p->cur_len = 0U;
    p->expected_len = 0U;
    p->head_match_pos = 0U;
    p->frame_start_tick = 0U;
    p->fifo_skip_count = 0U;
}

/**
 * @brief 快速帧头匹配
 * @param inst       协议解析器实例指针
 * @param fifo       kfifo指针
 * @param fifo_len   当前FIFO中可读字节数（由调用者传入，减少重复kfifo_len调用）
 * @param skip_bytes 输出：需要跳过的字节数（到第一个匹配字节的位置）
 * @retval 1 找到第一个帧头字节
 * @retval 0 未找到第一个帧头字节
 */
static uint8_t proto_try_match_head_fast(proto_t *inst,
                                         kfifo_t *fifo,
                                         unsigned int fifo_len,
                                         unsigned int *skip_bytes)
{
    const proto_frame_fmt_t *fmt = inst->fmt;
    unsigned int tail;
    unsigned int linear_size;
    const uint8_t *linear_buf;
    const uint8_t *found;

    *skip_bytes = 0U;

    /* 获取线性区域（不再在内部调用 kfifo_len，使用调用者传入的 fifo_len） */
    linear_size = kfifo_out_linear(fifo, &tail, fifo_len);
    if (linear_size == 0U) {
        return 0U;
    }

    linear_buf = (const uint8_t *)fifo->data + (tail * fifo->esize);

    /* 使用memchr快速查找帧头第一个字节 */
    found = (const uint8_t *)memchr(linear_buf, fmt->head[0], linear_size);

    if (found != NULL) {
        *skip_bytes = (unsigned int)(found - linear_buf);
        return 1U;
    }

    /* 未找到，跳过所有线性区域 */
    *skip_bytes = linear_size;
    return 0U;
}

/**
 * @brief 小长度帧尾比较（内联优化）
 * @param buf 缓冲区指针
 * @param tail 帧尾数据指针
 * @param len 比较长度（通常 <= 4）
 * @retval 0 匹配
 * @retval 非0 不匹配
 * @details 对于小长度（<=4字节）使用内联比较，避免 memcmp 函数调用开销
 */
static inline uint8_t proto_tail_cmp(const uint8_t *buf, const uint8_t *tail, uint8_t len)
{
    /* 小长度使用内联比较，减少函数调用开销 */
    if (len == 1U) {
        return (buf[0] != tail[0]) ? 1U : 0U;
    } else if (len == 2U) {
        return ((buf[0] != tail[0]) || (buf[1] != tail[1])) ? 1U : 0U;
    } else if (len == 3U) {
        return ((buf[0] != tail[0]) || (buf[1] != tail[1]) || (buf[2] != tail[2])) ? 1U : 0U;
    } else if (len == 4U) {
        return ((buf[0] != tail[0]) || (buf[1] != tail[1]) || 
                (buf[2] != tail[2]) || (buf[3] != tail[3])) ? 1U : 0U;
    } else {
        /* 长度大于4时回退到 memcmp */
        return (memcmp(buf, tail, (size_t)len) != 0) ? 1U : 0U;
    }
}

/**
 * @brief 在不移动 FIFO 指针的情况下读取指定偏移的数据
 * @param fifo   FIFO 实例指针
 * @param offset 从当前 out 偏移的字节数
 * @param dst    目标缓冲区
 * @param len    待读取的字节数
 * @retval 1 读取成功
 * @retval 0 读取失败（参数非法或FIFO不是按字节存储）
 */
static uint8_t proto_peek_fifo_bytes(kfifo_t *fifo,
                                     unsigned int offset,
                                     uint8_t *dst,
                                     unsigned int len)
{
    const uint8_t *buf;
    unsigned int   size;
    unsigned int   pos;
    unsigned int   first;

    if (len == 0U) {
        return 1U;
    }

    if ((fifo == NULL) || (dst == NULL) || (fifo->esize != 1U)) {
        return 0U;
    }

    buf  = (const uint8_t *)fifo->data;
    size = fifo->mask + 1U;

    pos   = (fifo->out + offset) & fifo->mask;
    first = (len < (size - pos)) ? len : (size - pos);

    (void)memcpy(dst, &buf[pos], first);

    if (len > first) {
        (void)memcpy(&dst[first], buf, len - first);
    }

    return 1U;
}

static uint8_t proto_append_from_fifo(proto_t *inst,
                                      kfifo_t *fifo,
                                      unsigned int fifo_len,
                                      uint16_t bytes_needed)
{
    uint16_t peek_offset = proto_get_fifo_offset(inst);
    unsigned int required = (unsigned int)peek_offset + (unsigned int)bytes_needed;

    if (bytes_needed == 0U) {
        return 1U;
    }

    if (fifo_len < required) {
        return 0U;
    }

    if ((inst->cur_len + bytes_needed) > inst->rx_buf_size) {
        proto_report_error(inst, (int)PROTO_ERR_BUF_TOO_SMALL);
        return 0U;
    }

    if (proto_peek_fifo_bytes(fifo,
                              peek_offset,
                              &inst->rx_buf[inst->cur_len],
                              bytes_needed) == 0U) {
        proto_report_error(inst, (int)PROTO_ERR_BUF_TOO_SMALL);
        return 0U;
    }

    inst->cur_len = (uint16_t)(inst->cur_len + bytes_needed);
    proto_update_tick(inst);
    return 1U;
}

/**
 * @brief 更新接收数据时间戳
 * @param p 协议解析器实例指针
 */
static inline void proto_update_tick(proto_t *p)
{
    if (p->get_tick != NULL) {
        p->frame_start_tick = p->get_tick();
    }
}

/**
 * @brief 报告错误并重置状态
 * @param p 协议解析器实例指针
 * @param err 错误码
 */
static inline void proto_report_error(proto_t *p, int err)
{
    if (p->on_error != NULL) {
        p->on_error(p, err);
    }
    proto_reset(p);
}

/**
 * @brief 检查缓冲区溢出
 * @param p 协议解析器实例指针
 * @retval 1 缓冲区溢出
 * @retval 0 缓冲区正常
 */
static inline uint8_t proto_check_buf_overflow(proto_t *p)
{
    return (p->cur_len >= p->rx_buf_size) ? 1U : 0U;
}

static inline uint16_t proto_get_fifo_offset(const proto_t *p)
{
    if (p->cur_len > p->head_match_pos) {
        return (uint16_t)(p->cur_len - p->head_match_pos);
    }
    return 0U;
}

/**
 * @brief 检查是否超时（数据流中断超时）
 * @param p 协议解析器实例指针
 * @retval 1 超时
 * @retval 0 未超时或未启用超时检测
 * @details 检测从上次收到数据到现在的时间，如果超过超时时间则判定为超时
 *          语义：数据流中断超时（FIFO为空且长时间无新数据）
 */
static inline uint8_t proto_check_timeout(proto_t *p)
{
    uint32_t elapsed;
    uint32_t current_tick;

    /* 如果未配置超时或未设置get_tick回调，直接返回 */
    if ((p->fmt->frame_timeout_ms == 0U) || (p->get_tick == NULL)) {
        return 0U;
    }

    /* 如果还没有开始接收帧，不检查超时 */
    if (p->frame_start_tick == 0U) {
        return 0U;
    }

    current_tick = p->get_tick();
    
    /* 处理回绕（优化：使用无符号减法自动处理回绕） */
    /* frame_start_tick 表示上次收到数据的时间 */
    elapsed = current_tick - p->frame_start_tick;

    return (elapsed > p->fmt->frame_timeout_ms) ? 1U : 0U;
}

/**
 * @brief 完成一帧的解析：校验CRC → 校验帧尾 → 回调
 * @param p 协议解析器实例指针
 * @details 解析完成后依次进行：
 *          1. CRC校验（如果配置了CRC）
 *          2. 帧尾校验（如果配置了帧尾）
 *          3. 调用帧接收完成回调
 *          4. 重置状态准备接收下一帧
 * @details 优化：使用嵌套if结构替代goto，符合MISRA规范
 */
static void proto_finish_frame(proto_t *p)
{
    const proto_frame_fmt_t *fmt = p->fmt;
    uint16_t frame_len = p->cur_len;
    uint16_t crc_pos;
    uint16_t calc_len;
    uint32_t calc;
    uint32_t got;
    uint8_t checksum_ok = 0U;

    /* 1) CRC校验 */
    if ((fmt->csum_cb != NULL) && (fmt->csum_size > 0U)) {
        /* 至少要有: tail_len + csum_size 这么多空间 */
        if (frame_len >= (uint16_t)(fmt->tail_len + fmt->csum_size)) {
            /* CRC字段起始下标（优化：避免重复计算） */
            crc_pos = (uint16_t)(frame_len - fmt->tail_len - fmt->csum_size);

            /* 起算偏移必须 < crc_pos（防止溢出） */
            if (fmt->csum_offset < crc_pos) {
                /* 计算：从csum_offset开始算到crc_pos之前 */
                calc_len = (uint16_t)(crc_pos - fmt->csum_offset);
                /* calc_len = crc_pos - csum_offset，且crc_pos < frame_len，因此csum_offset + calc_len = crc_pos < frame_len，无需额外检查 */
                calc = fmt->csum_cb(&p->rx_buf[fmt->csum_offset], calc_len);
                /* crc_pos + csum_size = frame_len - tail_len <= frame_len，无需额外检查 */
                got = read_u32(&p->rx_buf[crc_pos], fmt->csum_size, fmt->endian);
                
                if (calc == got) {
                    checksum_ok = 1U;
                }
            }
        }

        /* CRC校验失败 */
        if (checksum_ok == 0U) {
            proto_report_error(p, (int)PROTO_ERR_CHECKSUM);
            return;
        }
    } else {
        /* 未配置CRC，直接认为校验通过 */
        checksum_ok = 1U;
    }

    /* 2) 帧尾校验：只要配置了就校验 */
    if ((fmt->tail != NULL) && (fmt->tail_len > 0U)) {
        if (fmt->type != FRAME_TYPE_TAIL) {
            if ((frame_len < fmt->tail_len) ||
                (proto_tail_cmp(&p->rx_buf[frame_len - fmt->tail_len],
                               fmt->tail, fmt->tail_len) != 0U)) {
                proto_report_error(p, (int)PROTO_ERR_TAIL);
                return;
            }
        }
    }

    /* 3) 通知上层 */
    if (p->on_frame != NULL) {
        p->on_frame(p, p->rx_buf, frame_len);
    }

    /* 4) 标记完整帧已处理，可以跳过 FIFO 中的数据（保存到临时变量） */
    uint16_t skip_count;
    
    if (p->head_match_pos <= frame_len) {
        skip_count = (uint16_t)(frame_len - p->head_match_pos);
    } else {
        skip_count = frame_len;
    }
    
    /* 5) 准备接收下一帧 */
    proto_reset(p);
    
    /* 6) 恢复 skip_count（reset 后需要恢复，因为下一轮需要跳过） */
    p->fifo_skip_count = skip_count;
}

/**
 * @brief 向状态机喂入一个字节（仅用于TAIL型帧的逐字节扫描）
 * @param p 协议解析器实例指针
 * @param byte 待处理的字节
 * @details 仅处理PROTO_STATE_RECV_BODY状态下的TAIL型帧逐字节扫描
 */
static void proto_feed_byte(proto_t *p, uint8_t byte)
{
    const proto_frame_fmt_t *fmt = p->fmt;

    /* 更新上次收到数据的时间戳 */
    proto_update_tick(p);

    /* 仅处理RECV_BODY状态（TAIL型帧逐字节扫描时使用） */
    if (p->state == PROTO_STATE_RECV_BODY) {
        /* 检查缓冲区溢出 */
        if (proto_check_buf_overflow(p) != 0U) {
            proto_report_error(p, (int)PROTO_ERR_BUF_TOO_SMALL);
            return;
        }

        p->rx_buf[p->cur_len] = byte;
        p->cur_len++;

        /* TAIL型帧：检查是否达到最大长度 */
        if (p->cur_len > fmt->max_frame_len) {
            proto_report_error(p, (int)PROTO_ERR_LEN_FIELD);
            return;
        }

        /* 检查是否匹配帧尾 */
        if (p->cur_len >= fmt->tail_len) {
            if (proto_tail_cmp(&p->rx_buf[p->cur_len - fmt->tail_len],
                              fmt->tail, fmt->tail_len) == 0U) {
                proto_finish_frame(p);
            }
        }
    } else {
        /* 意外状态：不应该发生，重置状态机 */
        proto_reset(p);
    }
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化协议解析器实例
 * @param inst 协议解析器实例指针
 * @param fmt 帧格式描述指针
 * @param rx_buf 接收缓冲区指针
 * @param rx_buf_size 接收缓冲区大小
 * @param on_frame 帧接收完成回调函数（可为NULL）
 * @param on_error 错误回调函数（可为NULL）
 * @retval 0 成功
 * @retval -1 失败（参数无效）
 */
int proto_init(proto_t *inst,
               const proto_frame_fmt_t *fmt,
               uint8_t *rx_buf,
               uint16_t rx_buf_size,
               void (*on_frame)(proto_t *, const uint8_t *, uint16_t),
               void (*on_error)(proto_t *, int))
{
    if ((inst == NULL) || (fmt == NULL) || (rx_buf == NULL) || (rx_buf_size == 0U)) {
        return -1;
    }

    /* 检查帧格式参数有效性 */
    if (fmt->type == FRAME_TYPE_FIXED) {
        if (fmt->fixed_frame_len == 0U) {
            return -1;
        }
        if (fmt->fixed_frame_len > rx_buf_size) {
            return -1;
        }
        /* 检查固定长度帧是否足够容纳CRC和帧尾 */
        if ((fmt->csum_cb != NULL) && (fmt->csum_size > 0U)) {
            if (fmt->fixed_frame_len < (uint16_t)(fmt->tail_len + fmt->csum_size)) {
                return -1;
            }
            /* 检查CRC起算偏移是否合理 */
            uint16_t crc_pos = (uint16_t)(fmt->fixed_frame_len - fmt->tail_len - fmt->csum_size);
            if (fmt->csum_offset >= crc_pos) {
                return -1;
            }
        }
        /* 检查固定长度帧是否足够容纳帧尾 */
        if ((fmt->tail != NULL) && (fmt->tail_len > 0U)) {
            if (fmt->fixed_frame_len < fmt->tail_len) {
                return -1;
            }
        }
    } else if (fmt->type == FRAME_TYPE_LEN_FIELD) {
        if ((fmt->len_field_size != 1U) && (fmt->len_field_size != 2U) && (fmt->len_field_size != 4U)) {
            return -1;
        }
    } else if (fmt->type == FRAME_TYPE_TAIL) {
        if ((fmt->tail == NULL) || (fmt->tail_len == 0U)) {
            return -1;
        }
    }

    /* 检查校验和参数有效性（优化：合并条件判断） */
    if (fmt->csum_size != 0U) {
        if ((fmt->csum_size != 1U) && (fmt->csum_size != 2U) && (fmt->csum_size != 4U)) {
            return -1;
        }
        if (fmt->csum_cb == NULL) {
            return -1;
        }
    } else {
        if (fmt->csum_cb != NULL) {
            return -1;
        }
    }

    /* 检查最大帧长度参数有效性 */
    if (fmt->max_frame_len > rx_buf_size) {
        return -1;
    }
    
    /* 计算或验证最小帧长度 */
    uint16_t calc_min_len = (uint16_t)(fmt->head_len + fmt->tail_len + fmt->csum_size);
    if (fmt->min_frame_len == 0U) {
        /* 未配置，使用计算值 */
        if (fmt->type == FRAME_TYPE_FIXED) {
            calc_min_len = fmt->fixed_frame_len;
        }
    } else {
        /* 已配置，验证合理性 */
        if (fmt->min_frame_len < calc_min_len) {
            return -1;  /* 最小帧长不能小于协议开销 */
        }
        if (fmt->min_frame_len > fmt->max_frame_len) {
            return -1;  /* 最小帧长不能大于最大帧长 */
        }
        calc_min_len = fmt->min_frame_len;
    }
    
    /* 检查长度字段偏移是否会导致溢出 */
    if (fmt->type == FRAME_TYPE_LEN_FIELD) {
        if ((fmt->len_field_offset + fmt->len_field_size) > fmt->max_frame_len) {
            return -1;
        }
    }

    inst->fmt          = fmt;
    inst->rx_buf       = rx_buf;
    inst->rx_buf_size  = rx_buf_size;
    inst->on_frame     = on_frame;
    inst->on_error     = on_error;
    inst->get_tick     = NULL;
    inst->frame_start_tick = 0U;

    proto_reset(inst);
    return 0;
}

/**
 * @brief 设置获取时间戳回调函数（用于超时检测）
 * @param inst 协议解析器实例指针
 * @param get_tick 获取时间戳回调函数（可为NULL表示禁用超时检测）
 * @retval 0 成功
 * @retval -1 失败（参数无效）
 */
int proto_set_get_tick(proto_t *inst, proto_get_tick_cb_t get_tick)
{
    if (inst == NULL) {
        return -1;
    }

    inst->get_tick = get_tick;
    return 0;
}

/**
 * @brief 从kfifo轮询数据并解析（优化版：批量处理 + 快速帧头查找 + 数据预览）
 * @param inst 协议解析器实例指针
 * @param fifo kfifo指针
 * @retval >=0 成功处理的字节数
 * @retval -1 失败（参数无效）
 */
int proto_poll_from_kfifo(proto_t *inst, kfifo_t *fifo)
{
    unsigned int tail;
    unsigned int available;
    unsigned int fifo_len;
    const uint8_t *data_ptr;
    unsigned int i;
    unsigned int skip_bytes;
    uint16_t remain;
    int fed = 0;

    if ((inst == NULL) || (fifo == NULL)) {
        return -1;
    }

    /* 获取FIFO长度 */
    fifo_len = kfifo_len(fifo);

    /* 如果上一帧接收成功，跳过已确认的字节数 */
    if (inst->fifo_skip_count > 0U) {
        uint16_t skip = inst->fifo_skip_count;
        inst->fifo_skip_count = 0U;
        if (fifo_len >= skip) {
            kfifo_skip_count(fifo, skip);
            fifo_len -= skip;
            fed += (int)skip;
        } else {
            /* FIFO 中数据不足，等待下次 */
            kfifo_skip_count(fifo, fifo_len);
            fed += (int)fifo_len;
            inst->fifo_skip_count = (uint16_t)(skip - fifo_len);
            return fed;
        }
    }

    /* 超时检测 */
    if (inst->head_match_pos > 0U) {
        if (proto_check_timeout(inst) != 0U) {
            proto_report_error(inst, (int)PROTO_ERR_TIMEOUT);
            inst->head_match_pos = 0U;
            inst->cur_len        = 0U;
            inst->state          = PROTO_STATE_FIND_HEAD;
            inst->frame_start_tick = 0U;
            inst->expected_len   = 0U;
            return 0;
        }
    }

    while (fifo_len > 0U) {
        unsigned int old_fed = (unsigned int)fed;

        switch (inst->state) {
        case PROTO_STATE_FIND_HEAD: {
            /* 查找第一个帧头字节 */
            if (proto_try_match_head_fast(inst, fifo, fifo_len, &skip_bytes) != 0U) {
                /* 跳过前面的垃圾数据（确认不是帧头第一个字节的数据） */
                if (skip_bytes > 0U) {
                    kfifo_skip_count(fifo, skip_bytes);
                    fifo_len -= skip_bytes;
                    fed += (int)skip_bytes;
                }
                
                /* 找到第一个可能的帧头字节，只 peek，不 skip */
                if (inst->head_match_pos == 0U) {
                    inst->head_match_pos = 1U;
                    inst->cur_len        = 1U;
                    unsigned int peeked = kfifo_out_peek(fifo, inst->rx_buf, 1U);
                    if (peeked == 1U) {
                        /* 只 peek，不 skip，等待完整帧头验证 */
                        proto_update_tick(inst);
                    }
                }
                
                if (inst->fmt->head_len == 1U) {
                    /* 单字节帧头：proto_try_match_head_fast 已经验证了第一个字节匹配
                     * 现在可以跳过这个字节，并切换状态 */
                    kfifo_skip(fifo);
                    fifo_len--;
                    fed++;
                    
                    inst->cur_len        = 1U;
                    inst->head_match_pos = 1U;
                    
                    /* 根据帧类型切换状态 */
                    if (inst->fmt->type == FRAME_TYPE_FIXED) {
                        inst->expected_len = inst->fmt->fixed_frame_len;
                        inst->state        = PROTO_STATE_RECV_BODY;
                    } else if (inst->fmt->type == FRAME_TYPE_LEN_FIELD) {
                        inst->state = PROTO_STATE_READ_LEN;
                    } else { /* FRAME_TYPE_TAIL */
                        inst->state = PROTO_STATE_RECV_BODY;
                    }
                } else {
                    /* 多字节帧头：检查 FIFO 中剩余的数据是否足以补齐"整个帧头" */
                    if (fifo_len >= (inst->fmt->head_len - 1U)) {
                        /* 预览后续帧头字节，暂存到 rx_buf[1..head_len-1] */
                        unsigned int peeked = kfifo_out_peek(fifo,
                                                            &inst->rx_buf[inst->cur_len],
                                                            (unsigned int)(inst->fmt->head_len - inst->cur_len));

                        if ((peeked == (inst->fmt->head_len - inst->cur_len)) &&
                            (memcmp(inst->rx_buf, inst->fmt->head, inst->fmt->head_len) == 0)) {
                            /* 完整帧头匹配成功，现在可以跳过整个帧头 */
                            inst->cur_len        = inst->fmt->head_len;
                            inst->head_match_pos = inst->fmt->head_len;

                            proto_update_tick(inst);

                            /* 跳过 FIFO 中的整个帧头 */
                            kfifo_skip_count(fifo, inst->fmt->head_len);
                            fifo_len -= inst->fmt->head_len;
                            fed      += (int)inst->fmt->head_len;

                            /* 根据帧类型切换状态 */
                            if (inst->fmt->type == FRAME_TYPE_FIXED) {
                                inst->expected_len = inst->fmt->fixed_frame_len;
                                inst->state        = PROTO_STATE_RECV_BODY;
                            } else if (inst->fmt->type == FRAME_TYPE_LEN_FIELD) {
                                inst->state = PROTO_STATE_READ_LEN;
                            } else { /* FRAME_TYPE_TAIL */
                                inst->state = PROTO_STATE_RECV_BODY;
                            }
                        } else {
                            /* 帧头验证失败：丢掉当前"锁头"的第一个字节，重新找帧头 */
                            inst->head_match_pos  = 0U;
                            inst->cur_len         = 0U;
                            inst->frame_start_tick = 0U;

                            if (fifo_len > 0U) {
                                /* FIFO 里至少还有一个字节，丢弃一个继续找 */
                                kfifo_skip(fifo);
                                fifo_len--;
                                fed++;
                            }
                        }
                    }
                }
            } else {
                /* 如果之前已经在匹配一个可能的帧头，则直接放弃本次匹配 */
                if (inst->head_match_pos > 0U) {
                    inst->head_match_pos   = 0U;
                    inst->cur_len          = 0U;
                    inst->frame_start_tick = 0U;
                    
                    /* 之前 peek 的第一个字节不匹配，跳过它 */
                    if (fifo_len > 0U) {
                        kfifo_skip(fifo);
                        fifo_len--;
                        fed++;
                    }
                }

                /* 跳过整个线性区域（确认不是帧头第一个字节的数据） */
                if (skip_bytes > 0U) {
                    kfifo_skip_count(fifo, skip_bytes);
                    fifo_len -= skip_bytes;
                    fed      += (int)skip_bytes;
                }
            }
            break;
        }

        case PROTO_STATE_READ_LEN: {
            /* 对"带长度字段"的变长帧，先确保把 len 字段本身收齐 */
            uint16_t bytes_needed =
                (uint16_t)(inst->fmt->len_field_offset + inst->fmt->len_field_size);
            uint8_t can_continue = 1U;

            /* 还需要再读多少字节才能保证把长度字段读完整？ */
            if (inst->cur_len < bytes_needed) {
                uint16_t to_read = (uint16_t)(bytes_needed - inst->cur_len);

                if (proto_append_from_fifo(inst,
                                           fifo,
                                           fifo_len,
                                           to_read) == 0U) {
                    can_continue = 0U;
                }
            }
            
            if ((can_continue != 0U) && (inst->cur_len >= bytes_needed)) {
                uint32_t raw_len = read_u32(&inst->rx_buf[inst->fmt->len_field_offset],
                                            inst->fmt->len_field_size,
                                            inst->fmt->endian);
                uint16_t final_len;

                if (inst->fmt->len_cb != NULL) {
                    final_len = inst->fmt->len_cb(inst->rx_buf, (uint16_t)raw_len);
                } else {
                    final_len = (uint16_t)raw_len;
                }

                /* 长度合法性检查 */
                if ((final_len == 0U) ||
                    (final_len > inst->fmt->max_frame_len) ||
                    (final_len > inst->rx_buf_size)) {
                    proto_report_error(inst, (int)PROTO_ERR_LEN_FIELD);
                    can_continue = 0U;
                } else {
                    inst->expected_len = final_len;
                    inst->state        = PROTO_STATE_RECV_BODY;
                }
            }

            if (can_continue == 0U) {
                break;
            }
            break;
        }

        case PROTO_STATE_RECV_BODY: {
            const proto_frame_fmt_t *fmt = inst->fmt;
            uint8_t can_continue_recv = 1U;

            /* FIXED / LEN_FIELD 两种帧：可以批量拷贝数据 */
            if ((fmt->type == FRAME_TYPE_FIXED) || (fmt->type == FRAME_TYPE_LEN_FIELD)) {
                remain = inst->expected_len - inst->cur_len;

                if (remain > 0U) {
                    if (proto_append_from_fifo(inst,
                                               fifo,
                                               fifo_len,
                                               remain) == 0U) {
                        can_continue_recv = 0U;
                    } else if (inst->cur_len == inst->expected_len) {
                        proto_finish_frame(inst);
                    }
                }
            } else {
                /* TAIL 型帧：必须逐字节扫描帧尾，无法批量判定 */
                available = kfifo_out_linear(fifo, &tail, fifo_len);
                if (available > 0U) {
                    data_ptr = (const uint8_t *)fifo->data + (tail * fifo->esize);

                    for (i = 0U; i < available; i++) {
                        proto_feed_byte(inst, data_ptr[i]);
                        
                        /* 若 proto_feed_byte() 在内部完成了一帧，会调用 proto_reset()
                         * 这里无需额外判断，只是继续按照状态机处理下一帧。
                         */
                    }
                    
                    /* 只 peek，不 skip，等待完整帧接收成功 */
                }
            }

            if (can_continue_recv == 0U) {
                break;
            }
            break;
        }

        default:
            break;
        }

        /* 若本轮循环没有消费任何字节，则不再死循环 */
        if ((unsigned int)fed == old_fed) {
            break;
        }

        /* 更新 FIFO 长度，以便下一轮循环判断 */
        fifo_len = kfifo_len(fifo);

        /* 如果刚刚完成了一帧（proto_finish_frame 调用了 proto_reset），
         * 则状态会回到 FIND_HEAD，此时留给下一次调用处理新帧。
         */
        if (inst->state == PROTO_STATE_FIND_HEAD) {
            break;
        }

        if (fifo_len == 0U) {
            break;
        }
    }

    return fed;
}

/**
 * @brief 获取当前解析状态
 * @param inst 协议解析器实例指针（常量）
 * @return 当前解析状态，如果inst为NULL则返回PROTO_STATE_FIND_HEAD
 */
proto_state_t proto_get_state(const proto_t *inst)
{
    if (inst == NULL) {
        return PROTO_STATE_FIND_HEAD;
    }
    return inst->state;
}
