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
#include "proto_custom.h"
#include "custom_slave.h"
#include "data_mgmt.h"
#include "pulse_engine.h"
#include "major_logic.h"
#include "bsp_dac.h"
#include "stm32g4xx_hal.h"
#include "ocd.h"
#include "errno-base.h"
#include "board.h"
#include "gpio.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

/* Private function prototypes -----------------------------------------------*/

/* Exported variables  -------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 设置响应回调函数
 * @param cb 回调函数指针
 */
void cmd_handler_set_response_callback(cmd_response_cb_t cb)
{
    g_response_callback = cb;
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
    if (report.status == PULSE_STATUS_IDLE && report.count > 0U) {
        /* 空闲状态且计数>0表示已完成 */
        status_str = "OK";
    } else if (report.status == PULSE_STATUS_ERROR) {
        status_str = "ERROR";
    } else if (report.status == PULSE_STATUS_RUNNING) {
        status_str = "RUNNING";
    } else {
        status_str = "IDLE";
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

/**
 * @brief 处理获取软件版本命令
 */
void cmd_handle_get_software_version(const uint8_t *payload, uint16_t len, cmd_result_t *result)
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
void cmd_handle_set_hardware_version(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;

    if (len == 0U || len >= HW_VERSION_BUFSZ) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
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
void cmd_handle_get_hardware_version(const uint8_t *payload, uint16_t len, cmd_result_t *result)
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
void cmd_handle_set_serial_number(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;

    if (len == 0U || len >= SN_NUMBER_BUFSZ) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
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
void cmd_handle_get_serial_number(const uint8_t *payload, uint16_t len, cmd_result_t *result)
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
 * @brief 处理系统复位命令
 * @param payload 数据载荷（未使用）
 * @param len 数据载荷长度（未使用）
 * @param result 命令处理结果
 */
void cmd_handle_soft_reset(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    (void)payload;
    (void)len;

    result->ack_code = ACK_OK;
    result->resp_len = 0;

    /* 延时后执行软件复位（由应用层协调） */
    major_logic_request_reset();
    LOG_D("System reset requested");
}

/**
 * @brief 处理系统自检命令
 * @param payload 数据载荷（未使用）
 * @param len 数据载荷长度（未使用）
 * @param result 命令处理结果
 */
void cmd_handle_self_check(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    (void)payload;
    (void)len;

    /* 耗时命令先应答正在执行 */
    result->ack_code = ACK_IN_PROGERESS;
    result->resp_len = 0;

    /* TODO: 在后台任务中执行自检，完成后发送ACK_OK */
    LOG_D("Self check started");
}

/**
 * @brief 处理低功耗模式命令
 * @param payload 数据载荷（未使用）
 * @param len 数据载荷长度（未使用）
 * @param result 命令处理结果
 */
void cmd_handle_low_power_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    (void)payload;
    (void)len;

    result->ack_code = ACK_OK;
    result->resp_len = 0;

    /* TODO: 实现低功耗模式控制 */
    LOG_D("Low power mode command received");
}

/**
 * @brief 处理在线升级命令
 * @param payload 数据载荷（未使用）
 * @param len 数据载荷长度（未使用）
 * @param result 命令处理结果
 */
void cmd_handle_iap(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    (void)payload;
    (void)len;

    /* 耗时命令先应答正在执行 */
    result->ack_code = ACK_IN_PROGERESS;
    result->resp_len = 0;

    /* TODO: 在后台任务中执行IAP，完成后发送ACK_OK */
    LOG_D("IAP started");
}

/**
 * @brief 处理上传模式命令
 * @param payload 数据载荷（未使用）
 * @param len 数据载荷长度（未使用）
 * @param result 命令处理结果
 */
void cmd_handle_upload_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    (void)payload;
    (void)len;

    result->ack_code = ACK_OK;
    result->resp_len = 0;

    /* TODO: 实现上传模式控制 */
    LOG_D("Upload mode command received");
}

/**
 * @brief 处理状态上传命令
 * @param payload 数据载荷（未使用）
 * @param len 数据载荷长度（未使用）
 * @param result 命令处理结果
 */
void cmd_handle_status_upload(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    (void)payload;
    (void)len;

    /* 状态上传是非应答型命令，不需要回复 */
    result->ack_code = ACK_OK;
    result->resp_len = 0;

    LOG_D("Status upload received (no ACK required)");
}

/**
 * @brief 处理控制脉冲序列输出命令
 */
void cmd_handle_pulse_engine_start(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    uint8_t control;

    if (len != 1U) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
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
        result->ack_code = ACK_ERR_INVALID_PARAM;
    }

    result->resp_len = 0;
}

/**
 * @brief 处理获取脉冲引擎状态命令
 */
void cmd_handle_get_pulse_engine_status(const uint8_t *payload, uint16_t len, cmd_result_t *result)
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
void cmd_handle_set_pulse_engine_trigger_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    pulse_mode_t mode;

    if (len != 1U) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
        result->resp_len = 0;
        return;
    }

    mode = (pulse_mode_t)payload[0];

    ret = pulse_engine_set_mode(mode);
    if (ret == 0) {
        result->ack_code = ACK_OK;
        LOG_D("Set pulse mode: %d", mode);
    } else if (ret == -1) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
    } else if (ret == -2) {
        result->ack_code = ACK_ERR_BUSY;
    }

    result->resp_len = 0;
}

/**
 * @brief 处理获取脉冲输出模式命令
 */
void cmd_handle_get_pulse_engine_trigger_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result)
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
void cmd_handle_set_pulse_engine_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    pulse_params_t params;

    if (len != sizeof(pulse_params_t)) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
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
        result->ack_code = ACK_ERR_INVALID_PARAM;
        LOG_D("Failed to set pulse param");
    }

    result->resp_len = 0;
}

/**
 * @brief 处理获取脉冲序列参数命令
 */
void cmd_handle_get_pulse_engine_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result)
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
void cmd_handle_set_ecg_sync_trigger_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    ecg_sync_cfg_t config;

    if (len != sizeof(ecg_sync_cfg_t)) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
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
        result->ack_code = ACK_ERR_INVALID_PARAM;
        LOG_D("Failed to set ECG sync param");
    }

    result->resp_len = 0;
}

/**
 * @brief 处理获取心电同步触发参数命令
 */
void cmd_handle_get_ecg_sync_trigger_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result)
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

/**
 * @brief 处理设置过流检测电压阈值命令
 * @param payload 数据载荷
 * @param len 数据载荷长度
 * @param result 命令处理结果
 */
void cmd_handle_set_ocd_voltage_threshold(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    uint8_t channel;
    uint16_t voltage_mv;
    int ret;

    /* 数据格式：[通道(1字节)] [电压值(2字节，小端)] */
    if (len != 3U) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
        result->resp_len = 0;
        LOG_D("Invalid threshold param length: %d", len);
        return;
    }

    channel = payload[0];
    voltage_mv = (uint16_t)(payload[1] | (payload[2] << 8));

    /* 验证通道 */
    if (channel >= 2U) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
        result->resp_len = 0;
        LOG_D("Invalid channel: %d", channel);
        return;
    }

    /* 调用OCD模块设置阈值 */
    ret = ocd_set_threshold((ocd_ch_t)channel, voltage_mv);
    if (ret == 0) {
        result->ack_code = ACK_OK;
    } else {
        result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
    }

    result->resp_len = 0;
}

/**
 * @brief 处理获取过流检测电压阈值命令
 * @param payload 数据载荷
 * @param len 数据载荷长度
 * @param result 命令处理结果
 */
void cmd_handle_get_ocd_voltage_threshold(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    uint8_t channel;
    uint16_t voltage_mv;
    int ret;

    /* 数据格式：[通道(1字节)] */
    if (len != 1U) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
        result->resp_len = 0;
        LOG_D("Invalid param length: %d", len);
        return;
    }

    channel = payload[0];

    /* 验证通道 */
    if (channel >= 2U) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
        result->resp_len = 0;
        LOG_D("Invalid channel: %d", channel);
        return;
    }

    /* 调用OCD模块获取阈值 */
    ret = ocd_get_threshold((ocd_ch_t)channel, &voltage_mv);
    if (ret == 0) {
        /* 返回数据格式：[电压值(2字节，小端)] */
        result->resp_data[0] = (uint8_t)(voltage_mv & 0xFFU);
        result->resp_data[1] = (uint8_t)((voltage_mv >> 8) & 0xFFU);
        result->ack_code = ACK_OK;
        result->resp_len = 2U;
    } else {
        result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
        result->resp_len = 0;
    }
}

/**
 * @brief 处理读取硬件IO过流保护引脚状态命令
 * @param payload 数据载荷
 * @param len 数据载荷长度
 * @param result 命令处理结果
 */
void cmd_handle_get_ocp_pin_status(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    uint8_t pin_status;
    uint8_t positive_pin;
    uint8_t negative_pin;

    (void)payload;

    /* 数据格式：无参数 */
    if (len != 0U) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
        result->resp_len = 0;
        LOG_D("Invalid param length: %d", len);
        return;
    }

    /* 读取引脚状态 */
    pin_status = ocd_get_ocp_pin_status();
    positive_pin = gpio_read(OCP_POSITIVE_PIN_ID);
    negative_pin = gpio_read(OCP_NEGTIVE_PIN_ID);

    /* 返回数据格式：[安全状态(1字节)] [正脉冲引脚状态(1字节)] [负脉冲引脚状态(1字节)] */
    /* 安全状态：0=安全（两个引脚都为低），1=不安全（至少一个引脚为高） */
    result->resp_data[0] = pin_status;
    result->resp_data[1] = positive_pin;
    result->resp_data[2] = negative_pin;
    result->ack_code = ACK_OK;
    result->resp_len = 3U;

    LOG_D("Get OCP pin status: safe=%d, positive=%d, negative=%d",
          pin_status, positive_pin, negative_pin);
}

/**
 * @brief 处理硬件复位过流保护引脚命令
 * @param payload 数据载荷
 * @param len 数据载荷长度
 * @param result 命令处理结果
 */
void cmd_handle_reset_ocp_hardware(const uint8_t *payload, uint16_t len, cmd_result_t *result)
{
    int ret;
    uint8_t pin_status;

    (void)payload;

    /* 数据格式：无参数 */
    if (len != 0U) {
        result->ack_code = ACK_ERR_INVALID_PARAM;
        result->resp_len = 0;
        LOG_D("Invalid param length: %d", len);
        return;
    }

    /* 执行硬件复位（将复位引脚置高） */
    ret = ocd_reset();
    if (ret != 0) {
        result->ack_code = ACK_ERR_OPERATE_ABNORMAL;
        result->resp_len = 0;
        LOG_E("Failed to reset OCP hardware");
        return;
    }

    /* 读取复位后的引脚状态 */
    pin_status = ocd_get_ocp_pin_status();
    
    /* 解除脉冲锁定 */
    if (pin_status == 0U) {
        pulse_engine_ctrl_lock(0U);
    }

    /* 返回数据格式：[安全状态(1字节)] */
    /* 安全状态：0=安全（两个引脚都为低），1=不安全（至少一个引脚为高） */
    result->resp_data[0] = pin_status;
    result->ack_code = ACK_OK;
    result->resp_len = 1U;

    LOG_D("Reset OCP hardware: safe=%d", pin_status);
}

/* Private functions ---------------------------------------------------------*/

