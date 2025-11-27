/**
  ******************************************************************************
  * @file        : custom_slave.c
  * @brief       : 自定义协议从机实现
  * @details     本文件实现了custom_slave.h中定义的从机功能。
  *              职责：
  *              - 初始化协议解析器
  *              - 过滤设备地址
  *              - 处理握手和系统控制命令
  *              - 将业务命令分发给服务层
  * @attention   使用前需要先调用 slave_proto_init() 进行初始化
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "custom_slave.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "proto.h"
#include "proto_custom.h"
#include "kfifo.h"
#include "serial.h"
#include "errno-base.h"
#include "cmd_handler.h"
#include "current_monitor.h"
#include "bsp_dac.h"

#define  LOG_TAG             "custom_slave"
#define  LOG_LVL             4
#include "log.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

static serial_t *port;                                 /**< 串口设备指针 */
static proto_t g_slave_proto;                          /**< 协议解析器实例 */
static uint8_t g_slave_rx_buf[PROTO_CUSTOM_FRAME_MAX_LEN];  /**< 解析器内部组包用的临时buffer */
static uint8_t g_slave_valid_fifo_buf[8U * PROTO_CUSTOM_FRAME_MAX_LEN];   /**< 有效数据队列缓冲区 */
static kfifo_t g_slave_valid_fifo;                     /**< 有效数据输出队列 */
static volatile slave_state_t g_slave_state = SLAVE_STATE_WAIT_HANDSHAKE; /**< 从机当前状态 */
static uint32_t g_last_handshake_tick = 0U;  /**< 上次发送握手请求的时间戳 */

/* Private function prototypes -----------------------------------------------*/
static void slave_broadcast_handshake_request(void);
static void slave_process_handshake(void);
static inline int slave_build_frame(uint8_t *out, uint16_t out_size,
                                    uint8_t cmd, const uint8_t *data, 
                                    uint16_t data_len);
static void slave_proto_error_callback(void *inst, proto_err_t err);
static void slave_cmd_response_callback(uint8_t cmd, const cmd_result_t *result);
static inline int slave_build_response_frame(uint8_t *out, uint16_t out_size,
                                             uint8_t cmd, uint8_t ack,
                                             const uint8_t *data, 
                                             uint16_t data_len);
static void slave_send_response_frame(uint8_t cmd, const cmd_result_t *result);

/* Exported variables  -------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
extern uint32_t HAL_GetTick(void);

/**
 * @brief 初始化从机协议解析器
 * @retval 0 成功
 * @retval <0 失败（错误码）
 * @details 初始化串口设备、协议解析器和相关回调函数
 */
int slave_proto_init(void)
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

    /* 初始化命令处理服务 */
    cmd_handler_set_response_callback(slave_cmd_response_callback);

    /* 初始化有效数据输出队列 */
    ret = kfifo_init(&g_slave_valid_fifo,
                      g_slave_valid_fifo_buf,
                     (unsigned int)sizeof(g_slave_valid_fifo_buf),
                      1U);  /* 元素大小为1字节 */
    if (ret != 0) {
        LOG_D("Failed to initialize frame queue: %d\r\n", ret);
        return ret;
    }

    /* 初始化协议解析器（使用队列模式） */
    ret = proto_init(&g_slave_proto,
                     &CUSTOM_FMT,
                     g_slave_rx_buf,
                     (uint16_t)sizeof(g_slave_rx_buf),
                     &g_slave_valid_fifo,
                     slave_proto_error_callback);
    if (ret != 0) {
        LOG_D("Failed to initialize proto: %d\r\n", ret);
        return ret;
    }

    /* 设置时间戳回调函数，启用超时检测 */
    proto_set_tick_cb(&g_slave_proto, HAL_GetTick);
    return 0;
}
 
/**
 * @brief 轮询从机协议解析器
 * @details 从串口FIFO中读取数据并送入协议解析器处理。
 *          从帧队列中读取完整帧并处理。
 *          建议在主循环或任务中定期调用此函数。
 */
void slave_proto_task(void)
{
    uint8_t frame_buf[PROTO_CUSTOM_FRAME_MAX_LEN];
    unsigned int frame_len;

    if (port != NULL) {
        (void)proto_poll(&g_slave_proto, &port->rx_fifo);
    }

    /* 从输出队列中读取并处理帧 */
    while (kfifo_len(&g_slave_valid_fifo) > 0U) {
        frame_len = kfifo_out(&g_slave_valid_fifo, frame_buf, sizeof(frame_buf));
        if (frame_len > 0U) {
            slave_process_frame(frame_buf, (uint16_t)frame_len);
        }
    }

    /* 处理握手流程 */
    slave_process_handshake();
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 协议解析器错误回调函数
 * @param inst 协议解析器实例指针（void *类型，需转换为proto_t *）
 * @param err 错误码（proto_err_t）
 * @details 当协议解析器检测到帧解析错误时调用此函数，发送错误应答帧
 */
static void slave_proto_error_callback(void *inst, proto_err_t err)
{
    uint8_t frame[32U];
    int flen;
    uint8_t ack_code;
    
    (void)inst;
    
    if (port == NULL) {
        return;
    }
    
    /* 将协议解析器错误码转换为应答码 */
    switch (err) {
    case PROTO_ERR_LEN_INVALID:
        ack_code = ACK_ERR_LEN_FIELD;
        break;
    case PROTO_ERR_CHECKSUM:
        ack_code = ACK_ERR_CHECKSUM;
        break;
    case PROTO_ERR_TAIL:
        ack_code = ACK_ERR_TAIL;
        break;
    case PROTO_ERR_OUT_FIFO_FULL:
        ack_code = ACK_ERR_BUF_TOO_SMALL;
        break;
    case PROTO_ERR_TIMEOUT:
        ack_code = ACK_ERR_TIMEOUT;
        break;
    default:
        ack_code = ACK_ERR_UNKNOWN;
        break;
    }
    
    /* 构建并发送错误应答帧，命令码固定为0x2F */
    flen = slave_build_response_frame(frame, (uint16_t)sizeof(frame),
                                     CMD_FRAME_PARSER_ERR_ACK,  /* 帧解析错误应答命令码 */
                                     ack_code,                  /* 错误应答码 */
                                     NULL,                      /* 无数据载荷 */
                                     0U);
    
    if (flen > 0) {
        (void)serial_write(port, frame, (uint16_t)flen);
        LOG_D("Send frame parser error ACK: err=%d, ack=0x%02X", err, ack_code);
    }
}
 
/**
 * @brief 处理接收到的帧
 * @param frame 接收到的payload数据（已去除帧头、校验码、帧尾）
 * @param len payload长度
 * @details 解析帧内容，过滤产品地址和模块地址，并根据命令码分发处理
 * @note proto.c已经去除了帧头(0xFA)、校验码(CRC16)和帧尾(0x0D)，
 *       所以frame参数只包含：LEN(2) + DEV(1) + CMD(1) + MOD(1) + DATA(...)
 *       其中DEV为产品地址，MOD为模块地址，两者都必须匹配才会处理该帧
 */
void slave_process_frame(const uint8_t *frame, uint16_t len)
{
    /* frame结构（payload，已去除帧头、校验码、帧尾）：
    * [0..1]  = LEN（长度字段，小端）
    * [2]     = PRODUCT（产品地址，PRODUCT_ADDR）
    * [3]     = CMD（命令码）
    * [4]     = MOD（模块地址，MODULE_ADDR）
    * [5..]   = DATA（数据载荷，如果有）
    */
    /* 最小payload长度：LEN(2) + DEV(1) + CMD(1) + MOD(1) = 5字节 */
    if (len < 5U) {
        LOG_I("Frame too short: %d", (int)len);
        return;
    }

    uint8_t product = frame[2];      /* 产品地址 */
    uint8_t cmd = frame[3];
    uint8_t mod = frame[4];      /* 模块地址 */

    /* 地址检查：检查产品地址和模块地址，不匹配则需要应答错误 */
    if (product != PRODUCT_ADDR) {
        LOG_D("Product address mismatch: expected=0x%02X, got=0x%02X", 
              PRODUCT_ADDR, product);
        /* 发送产品地址错误应答 */
        cmd_result_t result;
        result.ack_code = ACK_ERR_PRODUCT_ADDR;
        result.resp_len = 0;
        slave_send_response_frame(cmd, &result);
        return;
    }
    
    if (mod != MODULE_ADDR) {
        LOG_D("Module address mismatch: expected=0x%02X, got=0x%02X", 
              MODULE_ADDR, mod);
        /* 发送模块地址错误应答 */
        cmd_result_t result;
        result.ack_code = ACK_ERR_MODULE_ADDR;
        result.resp_len = 0;
        slave_send_response_frame(cmd, &result);
        return;
    }

    /* 计算数据载荷长度 = payload总长 - 固定5字节（LEN+DEV+CMD+MOD） */
    uint16_t payload_len = (uint16_t)(len - 5U);
    const uint8_t *payload = &frame[5];

    /* 如果未握手，只处理握手命令，其他命令忽略 */
    if ((g_slave_state == SLAVE_STATE_WAIT_HANDSHAKE) && (cmd != CMD_HAND_SHAKE)) {
        LOG_I("Not handshaked yet, ignore cmd=0x%02X", cmd);
        return;
    }

    /* 根据命令类型处理 */
    cmd_result_t result;
    
    /* 初始化结果 */
    result.ack_code = ACK_OK;
    result.resp_len = 0;
    
    switch (cmd) {
    case CMD_HAND_SHAKE:
        if (g_slave_state == SLAVE_STATE_WAIT_HANDSHAKE) {
            /* 等待握手状态：收到主机的握手指令 */
            g_slave_state = SLAVE_STATE_ACTIVE;
            LOG_I("Handshake completed! Enter ACTIVE state");
        } else {
            /* 活动状态：响应主机的握手查询 */
            LOG_D("Handshake query in ACTIVE state");
        }
        
        slave_send_response_frame(CMD_HAND_SHAKE, &result);
        break;
    case CMD_GET_SOFTWARE_VERSION:
        cmd_handle_get_software_version(payload, payload_len, &result);
        slave_send_response_frame(CMD_GET_SOFTWARE_VERSION, &result);
        break;
    case CMD_SET_HARDWARE_VERSION:
        cmd_handle_set_hardware_version(payload, payload_len, &result);
        slave_send_response_frame(CMD_SET_HARDWARE_VERSION, &result);
        break;
    case CMD_GET_HARDWARE_VERSION:
        cmd_handle_get_hardware_version(payload, payload_len, &result);
        slave_send_response_frame(CMD_GET_HARDWARE_VERSION, &result);
        break;
    case CMD_SET_SERIAL_NUMBER:
        cmd_handle_set_serial_number(payload, payload_len, &result);
        slave_send_response_frame(CMD_SET_SERIAL_NUMBER, &result);
        break;
    case CMD_GET_SERIAL_NUMBER:
        cmd_handle_get_serial_number(payload, payload_len, &result);
        slave_send_response_frame(CMD_GET_SERIAL_NUMBER, &result);
        break;
    case CMD_CTRL_SOFT_RESET:
        cmd_handle_sys_reset(payload, payload_len, &result);
        slave_send_response_frame(CMD_CTRL_SOFT_RESET, &result);
        break;
    case CMD_CTRL_SELF_CHECK:
        cmd_handle_self_check(payload, payload_len, &result);
        slave_send_response_frame(CMD_CTRL_SELF_CHECK, &result);
        break;
    case CMD_CTRL_LOW_POWER_MODE:
        cmd_handle_low_power_mode(payload, payload_len, &result);
        slave_send_response_frame(CMD_CTRL_LOW_POWER_MODE, &result);
        break;
    case CMD_CTRL_IAP:
        cmd_handle_iap(payload, payload_len, &result);
        slave_send_response_frame(CMD_CTRL_IAP, &result);
        break;
    case CMD_CTRL_UPLOAD_MODE:
        cmd_handle_upload_mode(payload, payload_len, &result);
        slave_send_response_frame(CMD_CTRL_UPLOAD_MODE, &result);
        break;
    case CMD_STATUS_UPLOAD:
        cmd_handle_status_upload(payload, payload_len, &result);
        /* 状态上传是非应答型命令，不需要回复 */
        break;
    case CMD_SET_OCD_VOLTAGE_THRESHOLD:
        cmd_handle_set_ocd_voltage_threshold(payload, payload_len, &result);
        slave_send_response_frame(CMD_SET_OCD_VOLTAGE_THRESHOLD, &result);
        break;
    case CMD_GET_OCD_VOLTAGE_THRESHOLD:
        cmd_handle_get_ocd_voltage_threshold(payload, payload_len, &result);
        slave_send_response_frame(CMD_GET_OCD_VOLTAGE_THRESHOLD, &result);
        break;
    case CMD_CTRL_PULSE_ENGINE_START:
        cmd_handle_pulse_engine_start(payload, payload_len, &result);
        slave_send_response_frame(CMD_CTRL_PULSE_ENGINE_START, &result);
        break;
    case CMD_GET_PULSE_ENGINE_STATUS:
        cmd_handle_get_pulse_engine_status(payload, payload_len, &result);
        slave_send_response_frame(CMD_GET_PULSE_ENGINE_STATUS, &result);
        break;
    case CMD_SET_PULSE_ENGINE_TRIGGER_MODE:
        cmd_handle_set_pulse_engine_trigger_mode(payload, payload_len, &result);
        slave_send_response_frame(CMD_SET_PULSE_ENGINE_TRIGGER_MODE, &result);
        break;
    case CMD_GET_PULSE_ENGINE_TRIGGER_MODE:
        cmd_handle_get_pulse_engine_trigger_mode(payload, payload_len, &result);
        slave_send_response_frame(CMD_GET_PULSE_ENGINE_TRIGGER_MODE, &result);
        break;
    case CMD_SET_PULSE_ENGINE_PARAMETERS:
        cmd_handle_set_pulse_engine_parameters(payload, payload_len, &result);
        slave_send_response_frame(CMD_SET_PULSE_ENGINE_PARAMETERS, &result);
        break;
    case CMD_GET_PULSE_ENGINE_PARAMETERS:
        cmd_handle_get_pulse_engine_parameters(payload, payload_len, &result);
        slave_send_response_frame(CMD_GET_PULSE_ENGINE_PARAMETERS, &result);
        break;
    case CMD_SET_ECG_SYNC_TRIGGER_PARAMETERS:
        cmd_handle_set_ecg_sync_trigger_parameters(payload, payload_len, &result);
        slave_send_response_frame(CMD_SET_ECG_SYNC_TRIGGER_PARAMETERS, &result);
        break;
    case CMD_GET_ECG_SYNC_TRIGGER_PARAMETERS:
        cmd_handle_get_ecg_sync_trigger_parameters(payload, payload_len, &result);
        slave_send_response_frame(CMD_GET_ECG_SYNC_TRIGGER_PARAMETERS, &result);
        break;
    default:
        /* 未知命令 */
        if (g_slave_state == SLAVE_STATE_ACTIVE) {
            result.ack_code = ACK_ERR_INVALID_PARAM;
            result.resp_len = 0;
            slave_send_response_frame(cmd, &result);
        }
        break;
    }
}
 

/**
 * @brief 从机专用的命令帧构建函数（用于握手等命令帧）
 * @param out 输出缓冲区指针
 * @param out_size 输出缓冲区大小
 * @param cmd 命令码
 * @param data 数据载荷指针（可为NULL）
 * @param data_len 数据载荷长度
 * @retval >=0 构建的帧长度（字节数）
 * @retval -1 失败（缓冲区太小或参数无效）
 */
static inline int slave_build_frame(uint8_t *out, uint16_t out_size,
                                    uint8_t cmd, const uint8_t *data, 
                                    uint16_t data_len)
{
    return custom_build_frame(out, out_size,
                             PRODUCT_ADDR,      /* 使用固定产品地址 */
                             cmd,
                             MODULE_ADDR,      /* 使用固定模块地址 */
                             data, data_len);
}

/**
 * @brief 从机专用的应答帧构建函数
 * @param out 输出缓冲区指针
 * @param out_size 输出缓冲区大小
 * @param cmd 命令码
 * @param ack 应答码
 * @param data 数据载荷指针（可为NULL）
 * @param data_len 数据载荷长度
 * @retval >=0 构建的帧长度（字节数）
 * @retval -1 失败（缓冲区太小或参数无效）
 */
static inline int slave_build_response_frame(uint8_t *out, uint16_t out_size,
                                             uint8_t cmd, uint8_t ack,
                                             const uint8_t *data, 
                                             uint16_t data_len)
{
    return custom_build_response_frame(out, out_size,
                                      PRODUCT_ADDR,      /* 使用固定产品地址 */
                                      cmd,
                                      MODULE_ADDR,      /* 使用固定模块地址 */
                                      ack,
                                      data, data_len);
}

/**
 * @brief 构建并发送应答帧
 * @param cmd 命令码
 * @param result 命令处理结果
 * @details 根据命令处理结果构建应答帧并发送
 */
static void slave_send_response_frame(uint8_t cmd, const cmd_result_t *result)
{
    uint8_t frame[512U];
    int flen;
    
    if (result == NULL) {
        return;
    }
    
    /* 构建应答帧 */
    flen = slave_build_response_frame(frame, (uint16_t)sizeof(frame),
                                     cmd,
                                     result->ack_code,
                                     result->resp_data,
                                     result->resp_len);
    
    /* 发送应答帧 */
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
    }
}

/**
 * @brief 广播握手请求
 * @details 发送握手请求，等待主机响应
 */
static void slave_broadcast_handshake_request(void)
{
    uint8_t frame[32U];
    int flen;
    
    if (port == NULL) {
        return;
    }
    
    /* 使用便捷函数发送握手请求 */
    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_HAND_SHAKE,         /* 握手命令 */
                            NULL,                   /* 无数据 */
                            0U);
    
    if (flen > 0) {
        (void)serial_write(port, frame, (uint16_t)flen);
        LOG_D("Broadcast handshake request");
    }
}

/**
 * @brief 处理握手流程
 * @details 在等待握手状态下，周期性广播握手请求
 */
static void slave_process_handshake(void)
{
    uint32_t current_tick;
    uint32_t elapsed;
    
    /* 只在等待握手状态下广播 */
    if (g_slave_state != SLAVE_STATE_WAIT_HANDSHAKE) {
        return;
    }
    
    current_tick = HAL_GetTick();
    
    /* 处理时间戳回绕 */
    if (current_tick >= g_last_handshake_tick) {
        elapsed = current_tick - g_last_handshake_tick;
    } else {
        elapsed = (0xFFFFFFFFU - g_last_handshake_tick) + current_tick + 1U;
    }
    
    /* 每1秒广播一次 */
    if (elapsed >= HANDSHAKE_INTERVAL_MS) {
        slave_broadcast_handshake_request();
        g_last_handshake_tick = current_tick;
    }
}

/**
 * @brief 上报过流信息给上位机
 * @param info 过流信息指针
 */
void slave_upload_ocp_info(const ocp_info_t *info)
{
    uint8_t frame[64U];
    uint8_t upload_data[8U];
    int flen;
    
    if ((info == NULL) || (port == NULL)) {
        return;
    }
    
    /* 数据格式：[通道(1字节)] [峰值ADC值(2字节，小端)] [触发位置(2字节，小端)] [时间戳(4字节，小端)] */
    upload_data[0] = (uint8_t)info->channel;
    upload_data[1] = (uint8_t)(info->peak_value & 0xFFU);
    upload_data[2] = (uint8_t)((info->peak_value >> 8) & 0xFFU);
    upload_data[3] = (uint8_t)(info->trigger_position & 0xFFU);
    upload_data[4] = (uint8_t)((info->trigger_position >> 8) & 0xFFU);
    upload_data[5] = (uint8_t)(info->timestamp & 0xFFU);
    upload_data[6] = (uint8_t)((info->timestamp >> 8) & 0xFFU);
    upload_data[7] = (uint8_t)((info->timestamp >> 16) & 0xFFU);
    /* 注意：时间戳只发送低24位，高8位可能溢出，根据实际需求调整 */
    
    /* 使用状态上报命令码（或定义新的过流上报命令码） */
    /* 这里假设使用CMD_STATUS_UPLOAD，实际可能需要定义新的命令码 */
    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_STATUS_UPLOAD,  /* 或定义新的过流上报命令码 */
                            upload_data,
                            sizeof(upload_data));
    
    if (flen > 0) {
        (void)serial_write(port, frame, (uint16_t)flen);
        LOG_I("Upload OCP info: CH%d, peak=%d, pos=%d", 
              info->channel, info->peak_value, info->trigger_position);
    }
}

/**
 * @brief 命令处理服务的响应回调函数
 * @param cmd 命令码
 * @param result 命令处理结果
 */
static void slave_cmd_response_callback(uint8_t cmd, const cmd_result_t *result)
{
    uint8_t frame[512U];
    int flen;
    
    if (result == NULL) {
        return;
    }
    
    /* 构建并发送应答帧，应答码在帧结构中，数据载荷直接使用result->resp_data */
    flen = slave_build_response_frame(frame, (uint16_t)sizeof(frame),
                                     cmd,
                                     result->ack_code,         /* 应答码 */
                                     result->resp_data,        /* 数据载荷 */
                                     result->resp_len);         /* 数据长度 */
    
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
        LOG_D("Send response for cmd=0x%02X, ack=0x%02X", cmd, result->ack_code);
    }
}

/**
 * @brief 获取从机当前状态
 * @return 当前从机状态
 */
slave_state_t slave_get_state(void)
{
    return g_slave_state;
}

