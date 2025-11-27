/**
  ******************************************************************************
  * @file        : proto.c
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 20xx-xx-xx
  * @brief       : 通用协议解析器实现
  * @details     本文件实现了proto.h中定义的通用协议解析器功能。
  *              职责：
  *              - 从输入FIFO中读取原始数据
  *              - 根据配置的协议格式进行帧解析
  *              - 校验帧的完整性和正确性
  *              - 将解析完成的有效载荷推入输出FIFO
  *              - 支持固定长度、长度字段、尾部匹配三种帧类型
  * @attention   使用前需要先调用 proto_init() 进行初始化
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.实现通用协议解析器核心功能
  *                2.支持多种帧类型（固定长度、长度字段、尾部匹配）
  *                3.支持校验和验证和超时检测
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "proto.h"
#include <string.h>
#include "minmax.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define MAX_LOOP_COUNT          (8U)    /**< 最大循环次数限制（防止死循环） */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint32_t raw_read_uint(const uint8_t *p, uint8_t size, uint8_t is_be);
static int peek_byte_at(kfifo_t *fifo, unsigned int offset, uint8_t *byte);
static inline void reset_state(proto_t *inst);
static void report_err(proto_t *inst, proto_err_t err);
static inline void update_tick(proto_t *inst);
static void check_timeout(proto_t *inst, kfifo_t *fifo);
static void finalize_frame(proto_t *inst);
static int handle_find_head(proto_t *inst, kfifo_t *fifo);
static int handle_read_len(proto_t *inst, kfifo_t *fifo);
static int handle_recv_body(proto_t *inst, kfifo_t *fifo);
static int handle_match_tail(proto_t *inst, kfifo_t *fifo);

/* Exported variables  -------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化协议解析器
 * @param inst 协议解析器实例指针
 * @param cfg 协议配置指针（建议为const类型）
 * @param rx_buf 接收缓冲区指针
 * @param buf_size 接收缓冲区大小（必须 >= cfg->max_frame_len）
 * @param out_fifo 输出FIFO指针（解析完成的有效数据推入此队列）
 * @param on_err 错误回调函数指针（可为NULL）
 * @retval 0 成功
 * @retval -1 失败（参数无效或缓冲区太小）
 * @details 初始化协议解析器，设置配置、缓冲区和回调函数
 */
int proto_init(proto_t *inst, const proto_cfg_t *cfg, 
               uint8_t *rx_buf, uint16_t buf_size,
               kfifo_t *out_fifo, proto_error_cb_t on_err)
{
    if ((inst == NULL) || (cfg == NULL) || (rx_buf == NULL) || (out_fifo == NULL)) {
        return -1;
    }
    
    if (buf_size < cfg->max_frame_len) {
        return -1;
    }
    
    (void)memset(inst, 0, sizeof(proto_t));
    inst->cfg = cfg;
    inst->rx_buf = rx_buf;
    inst->buf_size = buf_size;
    inst->out_fifo = out_fifo;
    inst->on_error = on_err;
    
    reset_state(inst);
    return 0;
}

/**
 * @brief 设置时间戳回调函数
 * @param inst 协议解析器实例指针
 * @param get_tick 时间戳获取回调函数指针（可为NULL，禁用超时检测）
 * @details 用于超时检测，需要返回单调递增的时间戳（单位：毫秒）
 */
void proto_set_tick_cb(proto_t *inst, proto_tick_cb_t get_tick)
{
    if (inst != NULL) {
        inst->get_tick = get_tick;
    }
}

/**
 * @brief 核心轮询函数
 * @param inst 协议解析器实例指针
 * @param in_fifo 原始数据输入队列
 * @return 已处理的帧数（暂未实现计数，恒返回0，可忽略）
 * @details 从输入FIFO中读取数据，进行协议解析，将解析完成的有效数据推入输出FIFO。
 *          建议在主循环或任务中定期调用此函数。
 */
int proto_poll(proto_t *inst, kfifo_t *in_fifo)
{
    unsigned int loops = MAX_LOOP_COUNT;
    int ret;
    
    if ((inst == NULL) || (in_fifo == NULL)) {
        return 0;
    }
    
    check_timeout(inst, in_fifo);
    
    while ((kfifo_len(in_fifo) > 0U) && (loops > 0U)) {
        ret = 0;
        
        switch (inst->state) {
        case P_STATE_FIND_HEAD:
            ret = handle_find_head(inst, in_fifo);
            break;
        case P_STATE_READ_LEN:
            ret = handle_read_len(inst, in_fifo);
            break;
        case P_STATE_RECV_BODY:
            ret = handle_recv_body(inst, in_fifo);
            break;
        case P_STATE_MATCH_TAIL:
            ret = handle_match_tail(inst, in_fifo);
            break;
        default:
            reset_state(inst);
            break;
        }
        
        /* 如果状态机在等待数据(ret=0)，则退出循环，避免忙等待 */
        if (ret == 0) {
            break;
        }
        
        loops--;
    }
    
    return 0;
}

/**
 * @brief 重置解析器状态
 * @param inst 协议解析器实例指针
 * @details 将解析器状态重置为初始状态（寻找帧头），清空内部缓冲区索引
 */
void proto_reset(proto_t *inst)
{
    if (inst != NULL) {
        reset_state(inst);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 从字节数组中读取整数（支持大端/小端）
 * @param p 数据指针
 * @param size 字节数（1、2、4）
 * @param is_be 是否大端（1=大端，0=小端）
 * @return 读取的整数值
 * @details 简单的整数读取，避免依赖外部 byteorder 库
 */
static uint32_t raw_read_uint(const uint8_t *p, uint8_t size, uint8_t is_be)
{
    uint32_t val = 0U;
    uint8_t i;
    
    if (p == NULL) {
        return 0U;
    }
    
    if (is_be != 0U) {
        /* 大端：高位在前 */
        for (i = 0U; i < size; i++) {
            val = (val << 8U) | (uint32_t)p[i];
        }
    } else {
        /* 小端：低位在前 */
        for (i = 0U; i < size; i++) {
            val |= ((uint32_t)p[i] << (i * 8U));
        }
    }
    
    return val;
}

/**
 * @brief 查看FIFO中指定偏移的一个字节（无需拷贝整个缓冲区）
 * @param fifo FIFO指针
 * @param offset 偏移量
 * @param byte 输出字节指针
 * @retval 1 成功
 * @retval 0 失败（越界）
 * @details 直接利用 kfifo 结构体特性计算索引，无需拷贝数据
 */
static int peek_byte_at(kfifo_t *fifo, unsigned int offset, uint8_t *byte)
{
    unsigned int idx;
    
    if ((fifo == NULL) || (byte == NULL)) {
        return 0;
    }
    
    /* 检查是否越界：利用 kfifo_len */
    if (offset >= kfifo_len(fifo)) {
        return 0;
    }
    
    /* 直接利用 kfifo 结构体特性计算索引 */
    idx = (fifo->out + offset) & fifo->mask;
    *byte = ((uint8_t *)fifo->data)[idx];
    
    return 1;
}

/**
 * @brief 重置解析器状态
 * @param inst 协议解析器实例指针
 * @details 将状态重置为寻找帧头，清空写入索引和目标长度
 */
static inline void reset_state(proto_t *inst)
{
    if (inst != NULL) {
        inst->state = P_STATE_FIND_HEAD;
        inst->w_idx = 0U;
        inst->target_len = 0U;
    }
}

/**
 * @brief 报告错误并重置状态
 * @param inst 协议解析器实例指针
 * @param err 错误码
 * @details 调用错误回调函数（如果已设置），然后重置解析器状态
 */
static void report_err(proto_t *inst, proto_err_t err)
{
    if (inst != NULL) {
        if (inst->on_error != NULL) {
            inst->on_error(inst, err);
        }
        reset_state(inst);
    }
}

/**
 * @brief 更新时间戳
 * @param inst 协议解析器实例指针
 * @details 如果时间戳回调函数已设置，则更新最后接收字节的时间戳
 */
static inline void update_tick(proto_t *inst)
{
    if ((inst != NULL) && (inst->get_tick != NULL)) {
        inst->last_tick = inst->get_tick();
    }
}

/**
 * @brief 检查接收超时
 * @param inst 协议解析器实例指针
 * @param fifo 输入FIFO指针
 * @details 检查是否超时，如果超时则报告错误。
 *          关键修复：如果在 w_idx==0 时超时，说明数据卡在 FIFO 头部的预判阶段，
 *          必须跳过 1 个字节，打破死锁，让解析器有机会去扫描后面的数据。
 */
static void check_timeout(proto_t *inst, kfifo_t *fifo)
{
    bool is_rx_pending;
    bool is_fifo_pending;
    uint32_t current_tick;
    uint32_t elapsed;
    
    if ((inst == NULL) || (fifo == NULL) || (inst->cfg == NULL)) {
        return;
    }
    
    if ((inst->cfg->timeout_ms == 0U) || (inst->get_tick == NULL)) {
        return;
    }
    
    /* 状态判断 */
    is_rx_pending = (inst->w_idx > 0U);        /* rx_buf 里有数据（正在搬运中） */
    is_fifo_pending = (kfifo_len(fifo) > 0U);  /* FIFO 里有数据（可能卡在预判阶段） */
    
    /* 如果完全空闲（两边都没数据），更新时间戳保活，直接返回 */
    if ((!is_rx_pending) && (!is_fifo_pending)) {
        inst->last_tick = inst->get_tick();
        return;
    }
    
    /* 核心检查 */
    current_tick = inst->get_tick();
    
    /* 处理时间戳回绕 */
    if (current_tick >= inst->last_tick) {
        elapsed = current_tick - inst->last_tick;
    } else {
        elapsed = (0xFFFFFFFFU - inst->last_tick) + current_tick + 1U;
    }
    
    if (elapsed > inst->cfg->timeout_ms) {
        report_err(inst, PROTO_ERR_TIMEOUT);
        
        /* 关键修复：如果在 w_idx==0 时超时，说明数据卡在 FIFO 头部的预判阶段。
         * 必须跳过 1 个字节，打破死锁，让解析器有机会去扫描后面的数据。
         */
        if ((!is_rx_pending) && is_fifo_pending) {
            kfifo_skip(fifo);
        }
    }
}

/**
 * @brief 完成帧解析并输出有效载荷
 * @param inst 协议解析器实例指针
 * @details 对接收完整的帧进行校验（最小长度、尾部、校验和），
 *          然后提取有效载荷（不包含 Head, Tail, Checksum）并推入输出FIFO
 */
static void finalize_frame(proto_t *inst)
{
    const proto_cfg_t *cfg;
    uint16_t total_len;
    uint16_t csum_pos;
    uint32_t cal;
    uint32_t get;
    uint16_t payload_start;
    uint16_t payload_end;
    uint16_t payload_len;
    
    if ((inst == NULL) || (inst->cfg == NULL)) {
        return;
    }
    
    cfg = inst->cfg;
    total_len = inst->w_idx;
    
    /* 1. 最小长度检查 */
    if (total_len < cfg->min_frame_len) {
        reset_state(inst);
        return;
    }
    
    /* 2. 尾部校验 */
    if (cfg->tail_len > 0U) {
        if (memcmp(&inst->rx_buf[total_len - cfg->tail_len], 
                   cfg->tail, cfg->tail_len) != 0) {
            report_err(inst, PROTO_ERR_TAIL);
            return;
        }
    }
    
    /* 3. 校验和检查 */
    if ((cfg->csum_size > 0U) && (cfg->csum_cb != NULL)) {
        /* 假设校验位在 Payload 之后、Tail 之前 */
        csum_pos = total_len - cfg->tail_len - cfg->csum_size;
        
        /* 计算范围：从 csum_offset 开始，到 csum_pos 结束 */
        if (csum_pos > cfg->csum_offset) {
            cal = cfg->csum_cb(&inst->rx_buf[cfg->csum_offset], 
                               csum_pos - cfg->csum_offset);
            get = raw_read_uint(&inst->rx_buf[csum_pos], 
                               cfg->csum_size, 
                               cfg->is_big_endian);
            
            if (cal != get) {
                report_err(inst, PROTO_ERR_CHECKSUM);
                return;
            }
        }
    }
    
    /* 4. 提取有效 Payload 并推入输出 FIFO */
    /* 定义：Payload 不包含 Head, Tail, Checksum */
    payload_start = cfg->head_len;
    payload_end = total_len - cfg->tail_len - cfg->csum_size;
    
    if (payload_end > payload_start) {
        payload_len = payload_end - payload_start;
        
        /* 检查输出队列空间 */
        if (kfifo_avail(inst->out_fifo) >= payload_len) {
            /* 推入有效数据 */
            kfifo_in(inst->out_fifo, &inst->rx_buf[payload_start], payload_len);
        } else {
            report_err(inst, PROTO_ERR_OUT_FIFO_FULL);
        }
    }
    
    reset_state(inst);
}

/**
 * @brief 状态1：寻找帧头（包含尾部预测逻辑）
 * @param inst 协议解析器实例指针
 * @param fifo 输入FIFO指针
 * @retval 0 等待更多数据或状态切换
 * @retval 1 状态切换，需要立即处理
 * @details 在FIFO中查找帧头，对于LEN_FIELD类型还会进行长度和尾部预判
 */
static int handle_find_head(proto_t *inst, kfifo_t *fifo)
{
    const proto_cfg_t *cfg;
    unsigned int fifo_total;
    unsigned int tail_idx;
    unsigned int linear_cnt;
    const uint8_t *base_ptr;
    const uint8_t *found;
    unsigned int skip;
    uint16_t len_end;
    uint32_t raw_len;
    uint16_t frame_len;
    uint8_t tail_byte;
    
    if ((inst == NULL) || (fifo == NULL) || (inst->cfg == NULL)) {
        return 0;
    }
    
    cfg = inst->cfg;
    fifo_total = kfifo_len(fifo);
    
    /* 1. 获取线性连续内存块，用于 memchr 零拷贝查找 */
    linear_cnt = kfifo_out_linear(fifo, &tail_idx, fifo_total);
    base_ptr = (const uint8_t *)fifo->data + tail_idx;
    found = (const uint8_t *)memchr(base_ptr, cfg->head[0], linear_cnt);
    
    if (found == NULL) {
        /* 当前线性段内没找到，跳过这段数据 */
        kfifo_skip_count(fifo, linear_cnt);
        return 0;
    }
    
    /* 2. 跳过帧头前的垃圾数据 */
    skip = (unsigned int)(found - base_ptr);
    if (skip > 0U) {
        kfifo_skip_count(fifo, skip);
        fifo_total -= skip;
    }
    
    /* 3. 检查剩余数据是否足够匹配完整帧头 */
    if (fifo_total < cfg->head_len) {
        return 0; /* 数据不足，等待 */
    }
    
    /* 预览完整帧头到 rx_buf */
    if (kfifo_out_peek(fifo, inst->rx_buf, cfg->head_len) != cfg->head_len) {
        return 0;
    }
    
    if (memcmp(inst->rx_buf, cfg->head, cfg->head_len) != 0) {
        /* 第一个字节匹配但后续不匹配，只跳过第一个字节，重新寻找 */
        kfifo_skip(fifo);
        reset_state(inst);
        return 0;
    }
    
    /* 4. 长度与尾部预判（仅针对 LEN_FIELD 类型） */
    if (cfg->type == FRAME_TYPE_LEN_FIELD) {
        len_end = cfg->len_cfg.len_offset + cfg->len_cfg.len_size;
        
        /* 数据还不够解析长度字段，等待 */
        if (fifo_total < len_end) {
            return 0;
        }
        
        /* A. 提取长度字段 */
        (void)kfifo_out_peek(fifo, inst->rx_buf, len_end);
        raw_len = raw_read_uint(&inst->rx_buf[cfg->len_cfg.len_offset], 
                                cfg->len_cfg.len_size, 
                                cfg->is_big_endian);
        frame_len = (uint16_t)raw_len;
        if (cfg->len_cfg.len_cb != NULL) {
            frame_len = cfg->len_cfg.len_cb(inst->rx_buf, frame_len);
        }
        
        /* B. 长度范围检查（Min/Max） */
        if ((frame_len > cfg->max_frame_len) || (frame_len < cfg->min_frame_len)) {
            kfifo_skip(fifo); /* 这是一个坏头，必须跳过 */
            report_err(inst, PROTO_ERR_LEN_INVALID); /* 立即上报错误 */
            return 0;
        }
        
        /* 防止死锁 */
        if (frame_len > kfifo_size(fifo)) {
            kfifo_skip(fifo);
            report_err(inst, PROTO_ERR_LEN_INVALID); /* 立即上报错误 */
            return 0;
        }
        
        /* C. 尾部匹配检查（Look-Ahead，解决嵌套伪帧问题） */
        if ((fifo_total >= frame_len) && (cfg->tail_len > 0U)) {
            /* 偷看帧尾位置的字节 */
            if (peek_byte_at(fifo, frame_len - 1U, &tail_byte) != 0) {
                if (tail_byte != cfg->tail[cfg->tail_len - 1U]) {
                    /* 头对、长度对、但尾不对 -> 判定为帧尾错误 */
                    kfifo_skip(fifo);
                    report_err(inst, PROTO_ERR_TAIL);
                    return 0;
                }
            }
        }
        
        /* D. Store and Forward：数据不够全帧，先等待 */
        if (fifo_total < frame_len) {
            return 0;
        }
    }
    
    /* 5. 确认为真头，从 FIFO 搬运帧头数据 */
    (void)kfifo_out(fifo, inst->rx_buf, cfg->head_len);
    inst->w_idx = cfg->head_len;
    update_tick(inst);
    
    /* 状态迁移 */
    if (cfg->type == FRAME_TYPE_FIXED) {
        /* FIXED 模式使用 max_frame_len 作为固定长度 */
        inst->target_len = cfg->max_frame_len;
        inst->state = P_STATE_RECV_BODY;
    } else if (cfg->type == FRAME_TYPE_LEN_FIELD) {
        inst->state = P_STATE_READ_LEN;
    } else {
        inst->state = P_STATE_MATCH_TAIL;
    }
    
    return 0;
}

/**
 * @brief 状态2：读取长度字段
 * @param inst 协议解析器实例指针
 * @param fifo 输入FIFO指针
 * @retval 0 等待更多数据
 * @retval 1 状态切换，需要立即处理
 * @details 从FIFO中读取完整的长度字段，解析后进入接收帧体状态
 */
static int handle_read_len(proto_t *inst, kfifo_t *fifo)
{
    const proto_cfg_t *cfg;
    uint16_t needed;
    uint32_t raw;
    uint16_t final;
    
    if ((inst == NULL) || (fifo == NULL) || (inst->cfg == NULL)) {
        return 0;
    }
    
    cfg = inst->cfg;
    needed = (cfg->len_cfg.len_offset + cfg->len_cfg.len_size) - inst->w_idx;
    
    if (kfifo_len(fifo) < needed) {
        return 0;
    }
    
    /* 从 FIFO 读出数据 */
    (void)kfifo_out(fifo, &inst->rx_buf[inst->w_idx], needed);
    inst->w_idx += needed;
    update_tick(inst);
    
    raw = raw_read_uint(&inst->rx_buf[cfg->len_cfg.len_offset], 
                       cfg->len_cfg.len_size, 
                       cfg->is_big_endian);
    final = (uint16_t)raw;
    if (cfg->len_cfg.len_cb != NULL) {
        final = cfg->len_cfg.len_cb(inst->rx_buf, final);
    }
    
    /* 二次校验 */
    if ((final > cfg->max_frame_len) || (final < inst->w_idx)) {
        report_err(inst, PROTO_ERR_LEN_INVALID);
        return 0;
    }
    
    inst->target_len = final;
    inst->state = P_STATE_RECV_BODY;
    return 1; /* 状态切换，立即处理 */
}

/**
 * @brief 状态3：接收帧体
 * @param inst 协议解析器实例指针
 * @param fifo 输入FIFO指针
 * @retval 0 等待更多数据或帧接收完成
 * @details 从FIFO中批量接收帧体数据，直到达到目标长度
 */
static int handle_recv_body(proto_t *inst, kfifo_t *fifo)
{
    uint16_t remain;
    unsigned int copy_len;
    
    if ((inst == NULL) || (fifo == NULL)) {
        return 0;
    }
    
    remain = inst->target_len - inst->w_idx;
    /* 批量搬运 */
    copy_len = min(kfifo_len(fifo), remain);
    
    if (copy_len > 0U) {
        (void)kfifo_out(fifo, &inst->rx_buf[inst->w_idx], copy_len);
        inst->w_idx += copy_len;
        update_tick(inst);
    }
    
    if (inst->w_idx >= inst->target_len) {
        finalize_frame(inst);
    }
    
    return 0;
}

/**
 * @brief 状态4：匹配尾部（Tail模式，线性搜索优化版）
 * @param inst 协议解析器实例指针
 * @param fifo 输入FIFO指针
 * @retval 0 等待更多数据或帧接收完成
 * @details 使用 memchr 快速查找尾部最后一个字节，匹配完整尾部后完成帧解析
 */
static int handle_match_tail(proto_t *inst, kfifo_t *fifo)
{
    const proto_cfg_t *cfg;
    unsigned int fifo_len;
    unsigned int tail_idx;
    unsigned int linear_len;
    const uint8_t *base;
    uint8_t tail_last_byte;
    const uint8_t *found;
    unsigned int copy_len;
    
    if ((inst == NULL) || (fifo == NULL) || (inst->cfg == NULL)) {
        return 0;
    }
    
    cfg = inst->cfg;
    fifo_len = kfifo_len(fifo);
    
    linear_len = kfifo_out_linear(fifo, &tail_idx, fifo_len);
    if (linear_len == 0U) {
        return 0;
    }
    
    base = (const uint8_t *)fifo->data + tail_idx;
    tail_last_byte = cfg->tail[cfg->tail_len - 1U];
    
    /* 使用 memchr 快速查找尾部最后一个字节 */
    found = (const uint8_t *)memchr(base, tail_last_byte, linear_len);
    
    if (found == NULL) {
        /* 没找到尾，全部搬运到 rx_buf（如果 buffer 够大） */
        copy_len = linear_len;
        if ((inst->w_idx + copy_len) > inst->buf_size) {
            report_err(inst, PROTO_ERR_BUFF_OVERFLOW); /* 溢出 */
            return 0;
        }
        (void)kfifo_out(fifo, &inst->rx_buf[inst->w_idx], copy_len);
        inst->w_idx += copy_len;
        update_tick(inst);
        return 0;
    }
    
    /* 找到了潜在的尾部，只搬运到 found 的位置（包含 found） */
    copy_len = (unsigned int)(found - base) + 1U;
    
    if ((inst->w_idx + copy_len) > inst->buf_size) {
        report_err(inst, PROTO_ERR_LEN_INVALID);
        return 0;
    }
    
    (void)kfifo_out(fifo, &inst->rx_buf[inst->w_idx], copy_len);
    inst->w_idx += copy_len;
    update_tick(inst);
    
    /* 检查是否匹配完整尾部 */
    if (inst->w_idx >= cfg->tail_len) {
        if (memcmp(&inst->rx_buf[inst->w_idx - cfg->tail_len], 
                   cfg->tail, cfg->tail_len) == 0) {
            finalize_frame(inst);
            return 0;
        }
    }
    
    /* 如果匹配失败，继续等待后续数据（TAIL 模式容错机制） */
    return 0;
}
