/**
  ******************************************************************************
  * @file        : cmd_handler.c
  * @brief       : 命令处理服务实现
  * @details     本文件实现了cmd_handler.h中定义的命令处理功能。
  *              职责：
  *              - 根据命令码分发到具体处理函数
  *              - 调用业务模块接口
  *              - 构建统一格式的响应数据
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "cmd_handler.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "proto_custom.h"
#include "custom_slave.h"
#include "data_mgmt.h"
#include "pulse_engine.h"

#define  LOG_TAG             "cmd_handler"
#define  LOG_LVL             4
#include "log.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 响应回调函数指针
 */
static cmd_response_cb_t g_response_callback = NULL;

/**
 * @brief 上次的脉冲引擎状态（用于检测状态变化）
 */
static volatile pulse_status_t g_last_pulse_status = PULSE_STATUS_IDLE;

/* Private function prototypes -----------------------------------------------*/
static void handle_get_software_version(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_set_hardware_version(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_get_hardware_version(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_set_serial_number(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_get_serial_number(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_pulse_engine_start(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_get_pulse_engine_status(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_set_pulse_engine_trigger_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_get_pulse_engine_trigger_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_set_pulse_engine_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_get_pulse_engine_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_set_ecg_sync_trigger_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result);
static void handle_get_ecg_sync_trigger_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result);

/* Exported variables  -------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化命令处理服务
 */
void cmd_handler_init(void)
{
    g_response_callback = NULL;
    g_last_pulse_status = PULSE_STATUS_IDLE;
    
    LOG_D("Command handler initialized");
}

/**
 * @brief 设置响应回调函数
 * @param cb 回调函数指针
 */
void cmd_handler_set_response_callback(cmd_response_cb_t cb)
{
    g_response_callback = cb;
}

/**
 * @brief 处理命令
 * @param cmd 命令码
 * @param payload 数据载荷
 * @param len 数据长度
 */
void cmd_handler_process(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    cmd_result_t result;
    
    /* 初始化结果 */
    result.ack_code = ACK_OK;
    result.resp_len = 0;
    
    /* 根据命令码分发 */
    switch (cmd) {
    case CMD_GET_SOFTWARE_VERSION:
        handle_get_software_version(payload, len, &result);
        break;
    case CMD_SET_HARDWARE_VERSION:
        handle_set_hardware_version(payload, len, &result);
        break;
    case CMD_GET_HARDWARE_VERSION:
        handle_get_hardware_version(payload, len, &result);
        break;
    case CMD_SET_SERIAL_NUMBER:
        handle_set_serial_number(payload, len, &result);
        break;
    case CMD_GET_SERIAL_NUMBER:
        handle_get_serial_number(payload, len, &result);
        break;
    case CMD_CTRL_PULSE_ENGINE_START:
        handle_pulse_engine_start(payload, len, &result);
        break;
    case CMD_GET_PULSE_ENGINE_STATUS:
        handle_get_pulse_engine_status(payload, len, &result);
        break;
    case CMD_SET_PULSE_ENGINE_TRIGGER_MODE:
        handle_set_pulse_engine_trigger_mode(payload, len, &result);
        break;
    case CMD_GET_PULSE_ENGINE_TRIGGER_MODE:
        handle_get_pulse_engine_trigger_mode(payload, len, &result);
        break;
    case CMD_SET_PULSE_ENGINE_PARAMETERS:
        handle_set_pulse_engine_parameters(payload, len, &result);
        break;
    case CMD_GET_PULSE_ENGINE_PARAMETERS:
        handle_get_pulse_engine_parameters(payload, len, &result);
        break;
    case CMD_SET_ECG_SYNC_TRIGGER_PARAMETERS:
        handle_set_ecg_sync_trigger_parameters(payload, len, &result);
        break;
    case CMD_GET_ECG_SYNC_TRIGGER_PARAMETERS:
        handle_get_ecg_sync_trigger_parameters(payload, len, &result);
        break;
    default:
        LOG_I("Unknown cmd: 0x%02X", cmd);
        result.ack_code = ACK_ERR_DATA_INVALID_PARAM;
        result.resp_len = 0;
        break;
    }
    
    /* 调用响应回调 */
    if (g_response_callback != NULL) {
        g_response_callback(cmd, &result);
    }
}

/**
 * @brief 通知脉冲完成（由应用层调用）
 */
void cmd_handler_notify_pulse_complete(void)
{
    cmd_result_t result;
    pulse_report_t report;
    const char *mode_str;
    const char *status_str;
    uint16_t offset;
    
    /* 获取脉冲引擎状态 */
    if (pulse_engine_get_status(&report) != 0) {
        return;
    }
    
    /* 构建上报数据：| 模式字符串 | 状态字符串 | 次数(2字节) | */
    offset = 0;
    
    /* 模式字符串 */
    if (report.mode == PULSE_MODE_NORMAL) {
        mode_str = "Normal";
    } else {
        mode_str = "SYNC";
    }
    memcpy(&result.resp_data[offset], mode_str, strlen(mode_str));
    offset += strlen(mode_str);
    result.resp_data[offset++] = '|';
    
    /* 状态字符串 */
    if (report.status == PULSE_STATUS_COMPLETED) {
        status_str = "OK";
    } else if (report.status == PULSE_STATUS_ERROR) {
        status_str = "ERROR";
    } else {
        status_str = "RUNNING";
    }
    memcpy(&result.resp_data[offset], status_str, strlen(status_str));
    offset += strlen(status_str);
    result.resp_data[offset++] = '|';
    
    /* 次数(2字节) */
    result.resp_data[offset++] = (uint8_t)(report.count & 0xFFU);
    result.resp_data[offset++] = (uint8_t)((report.count >> 8) & 0xFFU);
    
    result.ack_code = ACK_OK;
    result.resp_len = offset;
    
    /* 调用响应回调 */
    if (g_response_callback != NULL) {
        g_response_callback(CMD_PULSE_ENGINE_STATUS_UPLOAD, &result);
    }
    
    LOG_D("Upload pulse status: %s|%s|%d", mode_str, status_str, report.count);
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 处理获取软件版本命令
 */
static void handle_get_software_version(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    const char *sw_version;
    uint16_t resp_len;

    (void)payload;
    (void)len;

    sw_version = data_mgmt_get_sw_version();
    LOG_D("SW VERSION = %s", sw_version);
    
    resp_len = (uint16_t)strlen(sw_version);
    memcpy(result->resp_data, sw_version, resp_len);
    result->resp_len = resp_len;
    result->ack_code = ACK_OK;
}

/**
 * @brief 处理设置硬件版本命令
 */
static void handle_set_hardware_version(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;

    if (len == 0U || len >= HW_VERSION_BUFSZ) {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
        result->resp_len = 0;
    } else {
        ret = data_mgmt_set_hw_version((const char *)payload, len);
        result->ack_code = (ret == 0) ? ACK_OK : ACK_ERR_OPERATE_ABNORMAL;
        result->resp_len = 0;
    }
}

/**
 * @brief 处理获取硬件版本命令
 */
static void handle_get_hardware_version(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    const char *hw_version;
    uint16_t resp_len;

    (void)payload;
    (void)len;

    hw_version = data_mgmt_get_hw_version();
    LOG_D("HW VERSION = %s", hw_version);
    
    resp_len = (uint16_t)strlen(hw_version);
    memcpy(result->resp_data, hw_version, resp_len);
    result->resp_len = resp_len;
    result->ack_code = ACK_OK;
}

/**
 * @brief 处理设置序列号命令
 */
static void handle_set_serial_number(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;

    if (len == 0U || len >= SN_NUMBER_BUFSZ) {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
        result->resp_len = 0;
    } else {
        ret = data_mgmt_set_sn_number((const char *)payload, len);
        result->ack_code = (ret == 0) ? ACK_OK : ACK_ERR_OPERATE_ABNORMAL;
        result->resp_len = 0;
    }
}

/**
 * @brief 处理获取序列号命令
 */
static void handle_get_serial_number(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    const char *sn_number;
    uint16_t resp_len;

    (void)payload;
    (void)len;

    sn_number = data_mgmt_get_sn_number();
    LOG_D("SN NUMBER = %s", sn_number);
    
    resp_len = (uint16_t)strlen(sn_number);
    memcpy(result->resp_data, sn_number, resp_len);
    result->resp_len = resp_len;
    result->ack_code = ACK_OK;
}

/**
 * @brief 处理控制脉冲序列输出命令
 */
static void handle_pulse_engine_start(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    uint8_t control;
    
    if (len != 1U) {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
        result->resp_len = 0;
        return;
    }
    
    control = payload[0];
    
    if (control == 0U) {
        /* 停止脉冲输出 */
        ret = pulse_engine_stop();
        if (ret == 0) {
            result->ack_code = ACK_OK;
            LOG_D("Pulse engine stopped");
        } else {
            result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
        }
    } else if (control == 1U) {
        /* 启动脉冲输出 */
        ret = pulse_engine_start();
        if (ret == 0) {
            result->ack_code = ACK_OK;
            LOG_D("Pulse engine started");
        } else {
            result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
            LOG_D("Failed to start pulse engine");
        }
    } else {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
    }
    
    result->resp_len = 0;
}

/**
 * @brief 处理获取脉冲引擎状态命令
 */
static void handle_get_pulse_engine_status(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    pulse_report_t report;

    (void)payload;
    (void)len;

    ret = pulse_engine_get_status(&report);
    if (ret == 0) {
        result->ack_code = ACK_OK;
        result->resp_data[0] = (uint8_t)report.mode;
        result->resp_data[1] = (uint8_t)report.status;
        result->resp_data[2] = (uint8_t)(report.count & 0xFFU);
        result->resp_data[3] = (uint8_t)((report.count >> 8) & 0xFFU);
        result->resp_len = 4U;
        LOG_D("Get pulse status: mode=%d, status=%d, count=%d", 
              report.mode, report.status, report.count);
    } else {
        result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
        result->resp_len = 0;
    }
}

/**
 * @brief 处理设置脉冲输出模式命令
 */
static void handle_set_pulse_engine_trigger_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    pulse_mode_t mode;

    if (len != 1U) {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
        result->resp_len = 0;
        return;
    }
    
    mode = (pulse_mode_t)payload[0];
    
    ret = pulse_engine_set_mode(mode);
    if (ret == 0) {
        result->ack_code = ACK_OK;
        LOG_D("Set pulse mode: %d", mode);
    } else {
        result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
    }
    
    result->resp_len = 0;
}

/**
 * @brief 处理获取脉冲输出模式命令
 */
static void handle_get_pulse_engine_trigger_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    pulse_mode_t mode;

    (void)payload;
    (void)len;

    ret = pulse_engine_get_mode(&mode);
    if (ret == 0) {
        result->ack_code = ACK_OK;
        result->resp_data[0] = (uint8_t)mode;
        result->resp_len = 1U;
        LOG_D("Get pulse mode: %d", mode);
    } else {
        result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
        result->resp_len = 0;
    }
}

/**
 * @brief 处理设置脉冲序列参数命令
 */
static void handle_set_pulse_engine_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    pulse_params_t params;

    if (len != sizeof(pulse_params_t)) {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
        result->resp_len = 0;
        LOG_D("Invalid param length: %d, expected: %d", len, sizeof(pulse_params_t));
        return;
    }
    
    memcpy(&params, payload, sizeof(pulse_params_t));
    
    ret = pulse_engine_set_seq_param(&params);
    if (ret == 0) {
        result->ack_code = ACK_OK;
        LOG_D("Set pulse param: group %d/%d", params.group_num, params.num_of_group);
    } else {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
        LOG_D("Failed to set pulse param");
    }
    
    result->resp_len = 0;
}

/**
 * @brief 处理获取脉冲序列参数命令
 */
static void handle_get_pulse_engine_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    pulse_params_t params[GROUP_NUM_MAX];
    uint16_t total_groups;

    (void)payload;
    (void)len;

    ret = pulse_engine_get_seq_param(params, &total_groups);
    if (ret == 0) {
        result->ack_code = ACK_OK;
        memcpy(result->resp_data, params, sizeof(pulse_params_t) * total_groups);
        result->resp_len = sizeof(pulse_params_t) * total_groups;
        LOG_D("Get pulse params: %d groups", total_groups);
    } else {
        result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
        result->resp_len = 0;
    }
}

/**
 * @brief 处理设置心电同步触发参数命令
 */
static void handle_set_ecg_sync_trigger_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    ecg_sync_cfg_t config;

    if (len != sizeof(ecg_sync_cfg_t)) {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
        result->resp_len = 0;
        LOG_D("Invalid ECG sync param length: %d, expected: %d", len, sizeof(ecg_sync_cfg_t));
        return;
    }
    
    memcpy(&config, payload, sizeof(ecg_sync_cfg_t));
    
    ret = pulse_engine_set_sync_param(&config);
    if (ret == 0) {
        result->ack_code = ACK_OK;
        LOG_D("Set ECG sync: interval=%d, delay=%lu, stop=%lu, repeat=%d", 
              config.interval_R, config.trigger_delay_ms, config.stop_delay_ms, config.repeat_count);
    } else {
        result->ack_code = ACK_ERR_DATA_INVALID_PARAM;
        LOG_D("Failed to set ECG sync param");
    }
    
    result->resp_len = 0;
}

/**
 * @brief 处理获取心电同步触发参数命令
 */
static void handle_get_ecg_sync_trigger_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    ecg_sync_cfg_t config;

    (void)payload;
    (void)len;

    ret = pulse_engine_get_sync_param(&config);
    if (ret == 0) {
        result->ack_code = ACK_OK;
        memcpy(result->resp_data, &config, sizeof(ecg_sync_cfg_t));
        result->resp_len = sizeof(ecg_sync_cfg_t);
        LOG_D("Get ECG sync: interval=%d, delay=%lu, stop=%lu, repeat=%d", 
              config.interval_R, config.trigger_delay_ms, config.stop_delay_ms, config.repeat_count);
    } else {
        result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
        result->resp_len = 0;
    }
}

