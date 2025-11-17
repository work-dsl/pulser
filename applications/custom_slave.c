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

#define HANDSHAKE_INTERVAL_MS  (1000U)              /**< 握手间隔：1秒（1Hz） */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 串口设备指针
 */
static serial_t *port;

/**
 * @brief 协议解析器实例
 */
static proto_t g_slave_proto;

/**
 * @brief 接收缓冲区
 */
static uint8_t g_slave_rx_buf[256U];

/**
 * @brief 从机当前状态
 */
static volatile slave_state_t g_slave_state = SLAVE_STATE_WAIT_HANDSHAKE;

/**
 * @brief 上次发送握手请求的时间戳
 */
static uint32_t g_last_handshake_broadcast_tick = 0U;

/* Private function prototypes -----------------------------------------------*/
static void slave_on_frame(proto_t *p, const uint8_t *frame, uint16_t len);
static void slave_on_error(proto_t *p, int err);
static uint32_t slave_get_tick(void);
static void slave_broadcast_handshake_request(void);
static void slave_process_handshake(void);
static inline int slave_build_frame(uint8_t *out, uint16_t out_size,
                                    uint8_t cmd, const uint8_t *data, 
                                    uint16_t data_len);
static void slave_handle_handshake(const uint8_t *payload, uint16_t len);
static void slave_handle_sys_reset(const uint8_t *payload, uint16_t len);
static void slave_handle_self_check(const uint8_t *payload, uint16_t len);
static void slave_handle_low_power_mode(const uint8_t *payload, uint16_t len);
static void slave_handle_iap(const uint8_t *payload, uint16_t len);
static void slave_handle_upload_mode(const uint8_t *payload, uint16_t len);
static void slave_handle_status_upload(const uint8_t *payload, uint16_t len);
static void slave_handle_set_ocd_voltage_threshold(const uint8_t *payload, uint16_t len);
static void slave_handle_get_ocd_voltage_threshold(const uint8_t *payload, uint16_t len);
static void slave_cmd_response_callback(uint8_t cmd, const cmd_result_t *result);

/* Exported variables  -------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

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
     cmd_handler_init();
     cmd_handler_set_response_callback(slave_cmd_response_callback);
     
     (void)proto_init(&g_slave_proto,
                &CUSTOM_FMT,
                g_slave_rx_buf,
                (uint16_t)sizeof(g_slave_rx_buf),
                slave_on_frame,
                slave_on_error);
     
     /* 设置时间戳回调函数，启用超时检测 */
     (void)proto_set_get_tick(&g_slave_proto, slave_get_tick);
                
     return 0;
 }
 
/**
 * @brief 轮询从机协议解析器
 * @details 从串口FIFO中读取数据并送入协议解析器处理。
 *          建议在主循环或任务中定期调用此函数。
 */
 void slave_proto_task(void)
 {
     if (port != NULL) {
         (void)proto_poll_from_kfifo(&g_slave_proto, &port->rx_fifo);
     }
     
     /* 处理握手流程 */
     slave_process_handshake();
 }

/* Private functions ---------------------------------------------------------*/
/**
 * @brief 获取系统时间戳（毫秒）
 * @return 系统时间戳（毫秒）
 * @details 可根据实际系统使用HAL_GetTick()或其他时间函数
 */
static uint32_t slave_get_tick(void)
{
    extern uint32_t HAL_GetTick(void);
    return HAL_GetTick();
}
 
/**
 * @brief 帧接收完成回调函数
 * @param p 协议解析器实例指针（未使用）
 * @param frame 接收到的完整帧数据
 * @param len 帧长度
 * @details 解析帧内容，过滤设备地址，并根据命令码分发处理
 */
static void slave_on_frame(proto_t *p, const uint8_t *frame, uint16_t len)
{
    (void)p;
    /* frame结构：
    * [0]     = 0xFA（帧头）
    * [1..2]  = LEN（长度字段，小端）
    * [3]     = DEV（设备地址）
    * [4]     = CMD（命令码）
    * [5]     = MOD（模块标识）
    * [6..]   = DATA（数据载荷）
    * [len-3] [len-2] = CRC16（校验，小端）
    * [len-1] = 0x0D（帧尾）
    */
    if (len < PROTO_CUSTOM_FRAME_MIN_LEN) {
        LOG_I("Frame too short: %d", (int)len);
        return;
    }

    uint8_t dev = frame[3];
    uint8_t cmd = frame[4];
    uint8_t mod = frame[5];
    
    LOG_D("Frame received: len=%d, dev=0x%02X, cmd=0x%02X, mod=0x%02X, state=%d", 
          (int)len, dev, cmd, mod, (int)g_slave_state);

    /* 地址过滤：不是给我的就直接丢弃 */
    if (dev != SLAVE_DEVICE_ADDR) {
        LOG_D("Address mismatch: expected=0x%02X, got=0x%02X", 
              SLAVE_DEVICE_ADDR, dev);
        return;
    }

    /* 计算payload长度 = 总长 - 固定9字节 */
    uint16_t payload_len = (uint16_t)(len - 9U);
    const uint8_t *payload = &frame[6];

    /* 如果未握手，只处理握手命令，其他命令忽略 */
    if ((g_slave_state == SLAVE_STATE_WAIT_HANDSHAKE) && (cmd != CMD_HAND_SHAKE)) {
        LOG_I("Not handshaked yet, ignore cmd=0x%02X", cmd);
        return;
    }

    /* 根据命令类型处理 */
    switch (cmd) {
    case CMD_HAND_SHAKE:
        slave_handle_handshake(payload, payload_len);
        break;
    case CMD_CTRL_SOFT_RESET:
        slave_handle_sys_reset(payload, payload_len);
        break;
    case CMD_CTRL_SELF_CHECK:
        slave_handle_self_check(payload, payload_len);
        break;
    case CMD_CTRL_LOW_POWER_MODE:
        slave_handle_low_power_mode(payload, payload_len);
        break;
    case CMD_CTRL_IAP:
        slave_handle_iap(payload, payload_len);
        break;
    case CMD_CTRL_UPLOAD_MODE:
        slave_handle_upload_mode(payload, payload_len);
        break;
    case CMD_STATUS_UPLOAD:
        slave_handle_status_upload(payload, payload_len);
        break;
    case CMD_SET_OCD_VOLTAGE_THRESHOLD:
        slave_handle_set_ocd_voltage_threshold(payload, payload_len);
        break;
    case CMD_GET_OCD_VOLTAGE_THRESHOLD:
        slave_handle_get_ocd_voltage_threshold(payload, payload_len);
        break;
    default:
        /* 其他命令交给命令处理服务 */
        if (g_slave_state == SLAVE_STATE_ACTIVE) {
            cmd_handler_process(cmd, payload, payload_len);
        }
        break;
    }
}
 
 /**
  * @brief 错误回调函数
  * @param p 协议解析器实例指针（未使用）
  * @param err 错误码
  */
static void slave_on_error(proto_t *p, int err)
{
    (void)p;
    
    switch (err) {
    case PROTO_ERR_TIMEOUT:
        LOG_I("Frame reception timeout, state=%d", (int)p->state);
        break;
    case PROTO_ERR_CHECKSUM:
        LOG_I("CRC checksum failed, state=%d, cur_len=%d", 
              (int)p->state, (int)p->cur_len);
        break;
    case PROTO_ERR_LEN_FIELD:
        LOG_I("Frame length errno, state=%d", (int)p->state);
        break;
    case PROTO_ERR_TAIL:
        LOG_I("Frame tail errno, state=%d", (int)p->state);
    default:
        LOG_I("Parser error: %d, state=%d", err, (int)p->state);
        break;
    }
}

/**
 * @brief 从机专用的帧构建函数
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
                             SLAVE_DEVICE_ADDR,      /* 使用固定设备地址 */
                             cmd,
                             SLAVE_MODULE_ADDR,      /* 使用固定模块地址 */
                             data, data_len);
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
    
    current_tick = slave_get_tick();
    
    /* 处理时间戳回绕 */
    if (current_tick >= g_last_handshake_broadcast_tick) {
        elapsed = current_tick - g_last_handshake_broadcast_tick;
    } else {
        elapsed = (0xFFFFFFFFU - g_last_handshake_broadcast_tick) + current_tick + 1U;
    }
    
    /* 每1秒广播一次 */
    if (elapsed >= HANDSHAKE_INTERVAL_MS) {
        slave_broadcast_handshake_request();
        g_last_handshake_broadcast_tick = current_tick;
    }
}

/**
 * @brief 处理握手命令
 * @param payload 数据载荷（未使用）
 * @param len 数据载荷长度（未使用）
 * @details 根据当前状态处理握手：
 *          - 等待握手状态：收到主机握手指令，回应并切换到活动状态
 *          - 活动状态：收到握手请求，简单回应
 */
static void slave_handle_handshake(const uint8_t *payload, uint16_t len)
{
    uint8_t resp_data[1] = {ACK_OK};
    uint8_t frame[32U];
    int flen;

    (void)payload;
    (void)len;

    if (g_slave_state == SLAVE_STATE_WAIT_HANDSHAKE) {
        /* 等待握手状态：收到主机的握手指令 */
        /* 停止广播，切换到活动状态 */
        g_slave_state = SLAVE_STATE_ACTIVE;
        LOG_I("Handshake completed! Enter ACTIVE state");
        
        /* 使用便捷函数回送握手应答 */
        flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                                CMD_HAND_SHAKE,         /* 握手应答 */
                                resp_data,
                                sizeof(resp_data));
        
        if ((flen > 0) && (port != NULL)) {
            (void)serial_write(port, frame, (uint16_t)flen);
            LOG_D("Send handshake ACK");
        }
    } else {
        /* 活动状态：响应主机的握手查询 */
        flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                                CMD_HAND_SHAKE,         /* 握手应答 */
                                resp_data,
                                sizeof(resp_data));
        
        if ((flen > 0) && (port != NULL)) {
            (void)serial_write(port, frame, (uint16_t)flen);
        }
    }
}

/**
 * @brief 处理系统复位命令
 * @param payload 数据载荷（未使用）
 * @param len 数据载荷长度（未使用）
 */
static void slave_handle_sys_reset(const uint8_t *payload, uint16_t len)
{
    uint8_t resp_data[1] = {ACK_OK};
    uint8_t frame[32U];
    int flen;

    (void)payload;
    (void)len;

    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_CTRL_SOFT_RESET,
                            resp_data,
                            sizeof(resp_data));
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
    }
    
    /* 延时后执行软件复位（由应用层协调） */
    extern void major_logic_request_reset(void);
    major_logic_request_reset();
}

static void slave_handle_self_check(const uint8_t *payload, uint16_t len)
{
    uint8_t resp_data[1] = {ACK_IN_PROGERESS};  /* 耗时命令先应答正在执行 */
    uint8_t frame[32U];
    int flen;

    (void)payload;
    (void)len;

    /* 使用便捷函数先应答ACK_IN_PROGERESS */
    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_CTRL_SELF_CHECK,
                            resp_data,
                            sizeof(resp_data));
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
    }
    
    /* TODO: 在后台任务中执行自检，完成后发送ACK_TASK_DONE */
}

static void slave_handle_low_power_mode(const uint8_t *payload, uint16_t len)
{
    uint8_t resp_data[1] = {ACK_OK};
    uint8_t frame[32U];
    int flen;

    (void)payload;
    (void)len;

    
    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_CTRL_LOW_POWER_MODE,
                            resp_data,
                            sizeof(resp_data));
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
    }
}

static void slave_handle_iap(const uint8_t *payload, uint16_t len)
{
    uint8_t resp_data[1] = {ACK_IN_PROGERESS};  /* 耗时命令先应答正在执行 */
    uint8_t frame[32U];
    int flen;

    (void)payload;
    (void)len;

    /* 使用便捷函数先应答ACK_IN_PROGERESS */
    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_CTRL_IAP,
                            resp_data,
                            sizeof(resp_data));
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
    }
    
    /* TODO: 在后台任务中执行IAP，完成后发送ACK_TASK_DONE */
}

static void slave_handle_upload_mode(const uint8_t *payload, uint16_t len)
{
    uint8_t resp_data[1] = {ACK_OK};
    uint8_t frame[32U];
    int flen;

    (void)payload;
    (void)len;

    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_CTRL_UPLOAD_MODE,
                            resp_data,
                            sizeof(resp_data));
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
    }
}

static void slave_handle_status_upload(const uint8_t *payload, uint16_t len)
{
    (void)payload;
    (void)len;
    
    /* 状态上传是非应答型命令，不需要回复 */
    LOG_D("Status upload received (no ACK required)");
}

/**
 * @brief 过流阈值存储（单位：mV）
 */
static uint16_t g_ocd_threshold_ch1 = 0U;  /**< 通道1阈值 */
static uint16_t g_ocd_threshold_ch2 = 0U;  /**< 通道2阈值 */

/**
 * @brief 电压转DAC值
 * @param voltage_mv 电压值（毫伏）
 * @return DAC寄存器值（12位，0-4095）
 * @note 假设VREF = 3.3V = 3300mV
 */
static uint16_t voltage_to_dac_value(uint16_t voltage_mv)
{
    /* DAC参考电压：3.3V = 3300mV */
    const uint16_t VREF_MV = 3300U;
    const uint16_t DAC_MAX = 4095U;
    uint32_t dac_value;
    
    /* 计算：DAC值 = (电压 / VREF) * 4095 */
    dac_value = ((uint32_t)voltage_mv * DAC_MAX) / VREF_MV;
    
    /* 限制在有效范围内 */
    if (dac_value > DAC_MAX) {
        dac_value = DAC_MAX;
    }
    
    return (uint16_t)dac_value;
}

/**
 * @brief DAC值转电压
 * @param dac_value DAC寄存器值（12位）
 * @return 电压值（毫伏）
 */
static uint16_t dac_value_to_voltage(uint16_t dac_value)
{
    const uint16_t VREF_MV = 3300U;
    const uint16_t DAC_MAX = 4095U;
    uint32_t voltage_mv;
    
    /* 计算：电压 = (DAC值 / 4095) * VREF */
    voltage_mv = ((uint32_t)dac_value * VREF_MV) / DAC_MAX;
    
    return (uint16_t)voltage_mv;
}

static void slave_handle_set_ocd_voltage_threshold(const uint8_t *payload, uint16_t len)
{
    uint8_t resp_data[1];
    uint8_t frame[32U];
    int flen;
    uint8_t channel;
    uint16_t voltage_mv;
    uint16_t dac_value;
    HAL_StatusTypeDef status;
    
    /* 数据格式：[通道(1字节)] [电压值(2字节，小端)] */
    if (len != 3U) {
        resp_data[0] = ACK_ERR_DATA_INVALID_PARAM;
        flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                                CMD_SET_OCD_VOLTAGE_THRESHOLD,
                                resp_data,
                                sizeof(resp_data));
        if ((flen > 0) && (port != NULL)) {
            (void)serial_write(port, frame, (uint16_t)flen);
        }
        LOG_D("Invalid threshold param length: %d", len);
        return;
    }
    
    channel = payload[0];
    voltage_mv = (uint16_t)(payload[1] | (payload[2] << 8));
    
    /* 验证通道 */
    if (channel >= 2U) {
        resp_data[0] = ACK_ERR_DATA_INVALID_PARAM;
        flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                                CMD_SET_OCD_VOLTAGE_THRESHOLD,
                                resp_data,
                                sizeof(resp_data));
        if ((flen > 0) && (port != NULL)) {
            (void)serial_write(port, frame, (uint16_t)flen);
        }
        LOG_D("Invalid channel: %d", channel);
        return;
    }
    
    /* 转换为DAC值 */
    dac_value = voltage_to_dac_value(voltage_mv);
    
    /* 设置DAC */
    if (channel == 0U) {
        /* 通道1（ADC1）对应COMP3，COMP3负端使用DAC1 */
        status = HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
        if (status == HAL_OK) {
            g_ocd_threshold_ch1 = voltage_mv;
            resp_data[0] = ACK_OK;
            LOG_D("Set CH1 (ADC1/COMP3) threshold: %d mV (DAC1=%d)", voltage_mv, dac_value);
        } else {
            resp_data[0] = ACK_ERR_OPERATE_ABNORMAL;
            LOG_E("Failed to set DAC1: %d", status);
        }
    } else {
        /* 通道2（ADC2）对应COMP1，COMP1负端使用DAC3 */
        status = HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
        if (status == HAL_OK) {
            g_ocd_threshold_ch2 = voltage_mv;
            resp_data[0] = ACK_OK;
            LOG_D("Set CH2 (ADC2/COMP1) threshold: %d mV (DAC3=%d)", voltage_mv, dac_value);
        } else {
            resp_data[0] = ACK_ERR_OPERATE_ABNORMAL;
            LOG_E("Failed to set DAC3: %d", status);
        }
    }
    
    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_SET_OCD_VOLTAGE_THRESHOLD,
                            resp_data,
                            sizeof(resp_data));
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
    }
}

static void slave_handle_get_ocd_voltage_threshold(const uint8_t *payload, uint16_t len)
{
    uint8_t resp_data[4];
    uint8_t frame[32U];
    int flen;
    uint8_t channel;
    
    /* 数据格式：[通道(1字节)] */
    if (len != 1U) {
        resp_data[0] = ACK_ERR_DATA_INVALID_PARAM;
        flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                                CMD_GET_OCD_VOLTAGE_THRESHOLD,
                                resp_data,
                                1U);
        if ((flen > 0) && (port != NULL)) {
            (void)serial_write(port, frame, (uint16_t)flen);
        }
        LOG_D("Invalid param length: %d", len);
        return;
    }
    
    channel = payload[0];
    
    /* 验证通道 */
    if (channel >= 2U) {
        resp_data[0] = ACK_ERR_DATA_INVALID_PARAM;
        flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                                CMD_GET_OCD_VOLTAGE_THRESHOLD,
                                resp_data,
                                1U);
        if ((flen > 0) && (port != NULL)) {
            (void)serial_write(port, frame, (uint16_t)flen);
        }
        LOG_D("Invalid channel: %d", channel);
        return;
    }
    
    /* 返回数据格式：[ACK(1字节)] [电压值(2字节，小端)] */
    resp_data[0] = ACK_OK;
    if (channel == 0U) {
        resp_data[1] = (uint8_t)(g_ocd_threshold_ch1 & 0xFFU);
        resp_data[2] = (uint8_t)((g_ocd_threshold_ch1 >> 8) & 0xFFU);
    } else {
        resp_data[1] = (uint8_t)(g_ocd_threshold_ch2 & 0xFFU);
        resp_data[2] = (uint8_t)((g_ocd_threshold_ch2 >> 8) & 0xFFU);
    }
    
    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            CMD_GET_OCD_VOLTAGE_THRESHOLD,
                            resp_data,
                            3U);
    if ((flen > 0) && (port != NULL)) {
        (void)serial_write(port, frame, (uint16_t)flen);
    }
    
    LOG_D("Get CH%d threshold: %d mV", channel, (channel == 0U) ? g_ocd_threshold_ch1 : g_ocd_threshold_ch2);
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
    uint8_t resp_data[256U];
    int flen;
    uint16_t resp_len;
    
    if (result == NULL) {
        return;
    }
    
    /* 构建响应数据：[ACK] + [结果数据] */
    resp_data[0] = result->ack_code;
    if (result->resp_len > 0) {
        memcpy(&resp_data[1], result->resp_data, result->resp_len);
        resp_len = 1U + result->resp_len;
    } else {
        resp_len = 1U;
    }
    
    /* 构建并发送响应帧 */
    flen = slave_build_frame(frame, (uint16_t)sizeof(frame),
                            cmd,
                            resp_data,
                            resp_len);
    
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

/**
 * @brief 重置从机到等待握手状态
 * @details 用于系统复位或重新初始化
 */
void slave_reset_to_handshake(void)
{
    g_slave_state = SLAVE_STATE_WAIT_HANDSHAKE;
    g_last_handshake_broadcast_tick = 0U;
    LOG_I("Reset to WAIT_HANDSHAKE state");
}

