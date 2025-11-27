/**
  ******************************************************************************
  * @file        : custom_host.c
  * @brief       : 自定义协议主机实现
  * @details     本文件实现了custom_host.h中定义的主机功能。
  *              职责：
  *              - 初始化协议解析器
  *              - 发送命令并等待响应
  *              - 处理从机响应
  *              - 超时管理
  * @attention   使用前需要先调用 host_proto_init() 进行初始化
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "custom_host.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "proto.h"
#include "proto_custom.h"
#include "kfifo.h"
#include "serial.h"
#include "errno-base.h"

#define  LOG_TAG             "custom_host"
#define  LOG_LVL             4
#include "log.h"

/* Private typedef -----------------------------------------------------------*/

/**
 * @brief 主机响应状态枚举
 */
typedef enum {
    HOST_RESP_NONE = 0,        /**< 无响应 */
    HOST_RESP_RECEIVED,        /**< 已收到响应 */
    HOST_RESP_TIMEOUT          /**< 响应超时 */
} host_resp_status_t;

/**
 * @brief 从机握手状态枚举
 */
typedef struct {
    uint8_t  device_addr;      /**< 从机设备地址 */
    uint8_t  is_online;        /**< 是否在线（已握手） */
    uint32_t last_seen_tick;   /**< 最后一次收到消息的时间 */
} slave_info_t;

/* Private define ------------------------------------------------------------*/

#define HOST_CMD_TIMEOUT_MS        (2000U)    /**< 命令响应超时时间（毫秒） */
#define HOST_MAX_RETRY             (3U)       /**< 命令最大重试次数 */
#define HOST_HANDSHAKE_TIMEOUT_MS  (1000U)    /**< 握手响应超时时间（毫秒） */
#define HOST_MAX_SLAVES            (8U)       /**< 最大从机数量 */
#define MIN(a, b)                  ((a) < (b) ? (a) : (b))  /**< 最小值宏 */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 串口设备指针
 */
static serial_t *port;

/**
 * @brief 协议解析器实例
 */
static proto_t g_host_proto;

/**
 * @brief 接收缓冲区
 */
static uint8_t g_host_rx_buf[256U];

/**
 * @brief 帧数据队列缓冲区
 * @details 队列中每个帧的格式：长度(2字节小端) + 帧数据
 *          最大帧长度64字节，队列可存储最多8帧
 */
static uint8_t g_host_frame_queue_buf[8U * (2U + 64U)];  /* 8帧 * (2字节长度 + 64字节数据) */

/**
 * @brief 帧数据队列
 */
static kfifo_t g_host_frame_queue;

/**
 * @brief 响应接收缓冲区
 */
static uint8_t g_host_resp_buf[256U];

/**
 * @brief 响应接收长度
 */
static volatile uint16_t g_host_resp_len = 0U;

/**
 * @brief 响应状态
 */
static volatile host_resp_status_t g_host_resp_status = HOST_RESP_NONE;

/**
 * @brief 命令发送时间戳
 */
static uint32_t g_host_cmd_tick = 0U;

/**
 * @brief 期待的响应命令码
 */
static uint8_t g_host_expect_cmd = 0U;

/**
 * @brief 从机信息列表
 */
static slave_info_t g_slave_list[HOST_MAX_SLAVES];

/**
 * @brief 从机数量
 */
static uint8_t g_slave_count = 0U;

/**
 * @brief 握手请求回调函数
 */
static void (*g_on_handshake_request_cb)(uint8_t dev) = NULL;

/* Private function prototypes -----------------------------------------------*/
static uint32_t host_get_tick(void);
static void host_process_frame(const uint8_t *frame, uint16_t len);
static int host_wait_response(uint32_t timeout_ms);
static void host_handle_handshake_request(uint8_t dev);
static int host_find_slave(uint8_t dev);

/* Exported variables  -------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 获取系统时间戳（毫秒）
 * @return 系统时间戳（毫秒）
 */
static uint32_t host_get_tick(void)
{
    extern uint32_t HAL_GetTick(void);
    return HAL_GetTick();
}

/**
 * @brief 查找从机索引
 * @param dev 从机设备地址
 * @return 从机索引，如果未找到返回-1
 */
static int host_find_slave(uint8_t dev)
{
    uint8_t i;
    
    for (i = 0U; i < g_slave_count; i++) {
        if (g_slave_list[i].device_addr == dev) {
            return (int)i;
        }
    }
    
    return -1;
}

/**
 * @brief 处理从机的握手请求（广播帧）
 * @param dev 从机设备地址（从帧中提取的源地址）
 * @details 主机侦听到从机广播的握手请求后，记录该从机并发送握手指令
 */
static void host_handle_handshake_request(uint8_t dev)
{
    int slave_idx;
    uint8_t frame[32U];
    int flen;
    
    /* 检查是否已存在 */
    slave_idx = host_find_slave(dev);
    
    if (slave_idx < 0) {
        /* 新设备上线 */
        if (g_slave_count < HOST_MAX_SLAVES) {
            g_slave_list[g_slave_count].device_addr = dev;
            g_slave_list[g_slave_count].is_online = 0U;
            g_slave_list[g_slave_count].last_seen_tick = host_get_tick();
            slave_idx = (int)g_slave_count;
            g_slave_count++;
            
            LOG_I("[HOST] New slave detected: 0x%02X", dev);
        } else {
            LOG_I("[HOST] Slave list full, cannot add: 0x%02X", dev);
            return;
        }
    }
    
    /* 发送握手指令 */
    flen = custom_build_frame(frame, (uint16_t)sizeof(frame),
                              dev,               /* 目标设备地址 */
                              CMD_HAND_SHAKE,    /* 握手指令 */
                              0x00U,            /* 模块标识 */
                              NULL,             /* 无数据 */
                              0U);
    
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
        LOG_I("[HOST] Send handshake command to slave 0x%02X", dev);
    }
    
    /* 调用用户回调 */
    if (g_on_handshake_request_cb != NULL) {
        g_on_handshake_request_cb(dev);
    }
}

/**
 * @brief 处理接收到的帧
 * @param frame 接收到的完整帧数据
 * @param len 帧长度
 * @details 接收从机响应，保存到响应缓冲区
 */
static void host_process_frame(const uint8_t *frame, uint16_t len)
{
    
    /* 检查帧有效性 */
    if (len >= 9U) {
        uint8_t dev = frame[3];
        uint8_t cmd = frame[4];
        
        /* 检查是否是广播的握手请求 */
        if (cmd == CMD_HAND_SHAKE) {
            /* 这是从机广播的握手请求，但我们无法从帧中直接获取源地址 */
            /* 需要从payload中解析，或者从机在广播时将自己的地址放在payload中 */
            /* 这里我们暂时不处理纯广播，等待从机响应主机的握手指令 */
            LOG_D("[HOST] Received handshake broadcast request");
            /* 触发握手流程 - 这里可以记录，等待从机响应时再建立链接 */
            return;
        }
        
        /* 如果是期待的响应命令或握手应答 */
        if ((g_host_expect_cmd == cmd) || (cmd == CMD_HAND_SHAKE)) {
            /* 保存响应数据 */
            if (len <= (uint16_t)sizeof(g_host_resp_buf)) {
                (void)memcpy(g_host_resp_buf, frame, len);
                g_host_resp_len = len;
                g_host_resp_status = HOST_RESP_RECEIVED;
                
                /* 如果是握手应答，更新从机状态 */
                if (cmd == CMD_HAND_SHAKE) {
                    int slave_idx = host_find_slave(dev);
                    if (slave_idx >= 0) {
                        g_slave_list[slave_idx].is_online = 1U;
                        g_slave_list[slave_idx].last_seen_tick = host_get_tick();
                        LOG_I("[HOST] Slave 0x%02X handshake completed!", dev);
                    }
                }
                
                LOG_D("[HOST] Response received, cmd=0x%02X, len=%d", cmd, len);
            }
        }
    }
}


/**
 * @brief 等待响应（带超时）
 * @param timeout_ms 超时时间（毫秒）
 * @retval 0 成功接收到响应
 * @retval -ETIMEDOUT 超时
 */
static int host_wait_response(uint32_t timeout_ms)
{
    uint32_t start_tick;
    uint32_t elapsed;
    uint32_t current_tick;
    
    start_tick = host_get_tick();
    g_host_resp_status = HOST_RESP_NONE;
    
        /* 轮询等待响应 */
    while (g_host_resp_status == HOST_RESP_NONE) {
        /* 轮询解析器（会自动处理队列中的帧） */
        host_proto_poll();
        
        /* 检查超时 */
        current_tick = host_get_tick();
        if (current_tick >= start_tick) {
            elapsed = current_tick - start_tick;
        } else {
            elapsed = (0xFFFFFFFFU - start_tick) + current_tick + 1U;
        }
        
        if (elapsed > timeout_ms) {
            LOG_I("[HOST] Wait response timeout");
            return -ETIMEOUT;
        }
    }
    
    return (g_host_resp_status == HOST_RESP_RECEIVED) ? 0 : -ETIMEOUT;
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化主机协议解析器
 * @retval 0 成功
 * @retval <0 失败（错误码）
 * @details 初始化串口设备、协议解析器和相关回调函数
 */
int host_proto_init(void)
{
    int ret;
    
    port = serial_find("uart1");
    if (port == NULL) {
        LOG_D("Failed to find uart1\r\n");
        return -ENODEV;
    }
    
    /* 初始化串口设备 */
    ret = serial_open(port);
    if (ret != 0) {
        LOG_D("Failed to initialize uart1: %d\r\n", ret);
        return ret;
    }
    
    /* 初始化帧数据队列 */
    ret = kfifo_init(&g_host_frame_queue,
                     g_host_frame_queue_buf,
                     (unsigned int)sizeof(g_host_frame_queue_buf),
                     1U);  /* 元素大小为1字节 */
    if (ret != 0) {
        LOG_D("Failed to initialize frame queue: %d\r\n", ret);
        return ret;
    }
    
    /* 初始化协议解析器（使用队列模式） */
    ret = proto_init(&g_host_proto,
                     &CUSTOM_FMT,
                     g_host_rx_buf,
                     (uint16_t)sizeof(g_host_rx_buf),
                     &g_host_frame_queue,
                     NULL);  /* 主机不需要错误回调 */
    if (ret != 0) {
        LOG_D("Failed to initialize proto: %d\r\n", ret);
        return ret;
    }
    
    /* 设置时间戳回调函数，启用超时检测 */
    proto_set_tick_cb(&g_host_proto, host_get_tick);
    
    /* 初始化响应状态 */
    g_host_resp_status = HOST_RESP_NONE;
    g_host_resp_len = 0U;
    g_host_expect_cmd = 0U;
               
    return 0;
}

/**
 * @brief 轮询主机协议解析器
 * @details 从串口FIFO中读取数据并送入协议解析器处理。
 *          从帧队列中读取完整帧并处理。
 *          建议在主循环或任务中定期调用此函数。
 */
void host_proto_poll(void)
{
    uint8_t frame_buf[256U];
    unsigned int frame_len;
    
    if (port != NULL) {
        (void)proto_poll(&g_host_proto, &port->rx_fifo);
    }
    
    /* 从输出队列中读取并处理帧 */
    while (kfifo_len(&g_host_frame_queue) > 0U) {
        frame_len = kfifo_out(&g_host_frame_queue, frame_buf, 
                             MIN((unsigned int)sizeof(frame_buf), 
                                 kfifo_len(&g_host_frame_queue)));
        if (frame_len > 0U) {
            host_process_frame(frame_buf, (uint16_t)frame_len);
        }
    }
}

/**
 * @brief 发送命令帧（带响应等待）
 * @param dev 目标设备地址
 * @param cmd 命令码
 * @param mod 模块标识
 * @param data 数据载荷指针（可为NULL）
 * @param data_len 数据载荷长度
 * @retval >=0 成功接收到响应的数据长度
 * @retval <0 失败（错误码）
 * @details 发送命令并等待响应，支持超时重试
 */
int host_send_cmd(uint8_t dev, uint8_t cmd, uint8_t mod,
                  const uint8_t *data, uint16_t data_len)
{
    uint8_t frame[256U];
    int flen;
    int ret;
    uint8_t retry;
    
    if (port == NULL) {
        return -ENODEV;
    }
    
    /* 构建命令帧 */
    flen = custom_build_frame(frame, (uint16_t)sizeof(frame),
                              dev, cmd, mod,
                              data, data_len);
    if (flen <= 0) {
        LOG_I("[HOST] Failed to build frame");
        return -EINVAL;
    }
    
    /* 设置期待的响应命令 */
    g_host_expect_cmd = cmd;
    
    /* 重试发送 */
    for (retry = 0U; retry < HOST_MAX_RETRY; retry++) {
        /* 清空响应状态 */
        g_host_resp_status = HOST_RESP_NONE;
        g_host_resp_len = 0U;
        
        /* 发送命令 */
        g_host_cmd_tick = host_get_tick();
        ret = serial_write(port, frame, (uint16_t)flen);
        if (ret < 0) {
            LOG_I("[HOST] Failed to send command, retry=%d", retry);
            continue;
        }
        
        LOG_D("[HOST] Command sent, cmd=0x%02X, len=%d, retry=%d", cmd, flen, retry);
        
        /* 等待响应 */
        ret = host_wait_response(HOST_CMD_TIMEOUT_MS);
        if (ret == 0) {
            /* 成功接收响应 */
            LOG_D("[HOST] Command success, response_len=%d", g_host_resp_len);
            return (int)g_host_resp_len;
        }
        
        LOG_I("[HOST] No response, retry=%d", retry);
    }
    
    /* 所有重试都失败 */
    LOG_I("[HOST] Command failed after %d retries", HOST_MAX_RETRY);
    return -ETIMEOUT;
}

/**
 * @brief 获取最后一次接收的响应数据
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @retval >=0 响应数据长度
 * @retval -1 无响应数据或缓冲区太小
 */
int host_get_last_response(uint8_t *buf, uint16_t buf_size)
{
    if ((buf == NULL) || (g_host_resp_len == 0U) || (buf_size < g_host_resp_len)) {
        return -1;
    }
    
    (void)memcpy(buf, g_host_resp_buf, g_host_resp_len);
    return (int)g_host_resp_len;
}

/**
 * @brief 主动发送握手指令给指定从机
 * @param dev 目标从机设备地址
 * @retval 0 成功
 * @retval <0 失败（错误码）
 * @details 主机主动向指定从机发送握手指令并等待响应
 */
int host_send_handshake(uint8_t dev)
{
    uint8_t frame[32U];
    int flen;
    int ret;
    int slave_idx;
    
    if (port == NULL) {
        return -ENODEV;
    }
    
    /* 检查从机是否已存在 */
    slave_idx = host_find_slave(dev);
    if (slave_idx < 0) {
        /* 新从机，添加到列表 */
        if (g_slave_count < HOST_MAX_SLAVES) {
            g_slave_list[g_slave_count].device_addr = dev;
            g_slave_list[g_slave_count].is_online = 0U;
            g_slave_list[g_slave_count].last_seen_tick = host_get_tick();
            slave_idx = (int)g_slave_count;
            g_slave_count++;
        } else {
            return -ENOMEM;
        }
    }
    
    /* 构建握手帧 */
    flen = custom_build_frame(frame, (uint16_t)sizeof(frame),
                              dev,               /* 目标设备地址 */
                              CMD_HAND_SHAKE,    /* 握手指令 */
                              0x00U,            /* 模块标识 */
                              NULL,             /* 无数据 */
                              0U);
    
    if (flen <= 0) {
        return -EINVAL;
    }
    
    /* 设置期待的响应命令 */
    g_host_expect_cmd = CMD_HAND_SHAKE;
    g_host_resp_status = HOST_RESP_NONE;
    g_host_resp_len = 0U;
    
    /* 发送握手指令 */
    ret = serial_write(port, frame, (uint16_t)flen);
    if (ret < 0) {
        return ret;
    }
    
    LOG_I("[HOST] Send handshake to slave 0x%02X", dev);
    
    /* 等待响应 */
    ret = host_wait_response(HOST_HANDSHAKE_TIMEOUT_MS);
    if (ret == 0) {
        /* 握手成功 */
        g_slave_list[slave_idx].is_online = 1U;
        g_slave_list[slave_idx].last_seen_tick = host_get_tick();
        LOG_I("[HOST] Handshake with slave 0x%02X completed!", dev);
        return 0;
    }
    
    LOG_I("[HOST] Handshake with slave 0x%02X failed", dev);
    return -ETIMEOUT;
}

/**
 * @brief 获取从机在线状态
 * @param dev 从机设备地址
 * @retval 1 从机在线
 * @retval 0 从机离线或未知
 */
int host_is_slave_online(uint8_t dev)
{
    int slave_idx = host_find_slave(dev);
    
    if (slave_idx < 0) {
        return 0;
    }
    
    return (int)g_slave_list[slave_idx].is_online;
}

/**
 * @brief 获取已连接的从机数量
 * @return 从机数量
 */
uint8_t host_get_slave_count(void)
{
    return g_slave_count;
}

/**
 * @brief 获取从机列表
 * @param buf 输出缓冲区（存放设备地址）
 * @param buf_size 缓冲区大小
 * @retval >=0 实际从机数量
 * @retval -1 参数错误
 */
int host_get_slave_list(uint8_t *buf, uint8_t buf_size)
{
    uint8_t i;
    uint8_t count = 0U;
    
    if (buf == NULL) {
        return -1;
    }
    
    for (i = 0U; (i < g_slave_count) && (count < buf_size); i++) {
        buf[count] = g_slave_list[i].device_addr;
        count++;
    }
    
    return (int)count;
}

/**
 * @brief 设置握手请求回调函数
 * @param cb 回调函数指针
 * @details 当主机收到从机的握手请求时会调用此回调
 */
void host_set_handshake_callback(void (*cb)(uint8_t dev))
{
    g_on_handshake_request_cb = cb;
}

