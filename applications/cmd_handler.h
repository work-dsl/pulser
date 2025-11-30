/**
  ******************************************************************************
  * @file        : cmd_handler.h
  * @brief       : 命令处理服务接口
  * @details     本文件定义了命令处理服务的接口函数。
  *              职责：
  *              - 接收协议层传来的命令
  *              - 调用对应的业务模块处理命令
  *              - 构建响应数据
  * @attention   使用前需要先调用 cmd_handler_init() 进行初始化
  ******************************************************************************
  */

#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "custom_slave.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 命令处理结果结构体
 */
typedef struct {
    uint8_t  ack_code;          /**< 应答码 */
    uint8_t  resp_data[256];    /**< 响应数据缓冲区 */
    uint16_t resp_len;          /**< 响应数据长度 */
} cmd_result_t;

/**
 * @brief 命令处理回调函数类型
 * @param result 命令处理结果指针
 */
typedef void (*cmd_response_cb_t)(uint8_t cmd, const cmd_result_t *result);

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void cmd_handler_set_response_callback(cmd_response_cb_t cb);
void cmd_handler_notify_pulse_complete(void);
void cmd_handle_sys_reset(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_self_check(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_low_power_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_iap(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_upload_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_status_upload(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_get_software_version(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_set_hardware_version(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_get_hardware_version(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_set_serial_number(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_get_serial_number(const uint8_t *payload, uint16_t len, cmd_result_t *result);

void cmd_handle_pulse_engine_start(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_get_pulse_engine_status(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_set_pulse_engine_trigger_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_get_pulse_engine_trigger_mode(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_set_pulse_engine_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_get_pulse_engine_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_set_ecg_sync_trigger_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_get_ecg_sync_trigger_parameters(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_set_ocd_voltage_threshold(const uint8_t *payload, uint16_t len, cmd_result_t *result);
void cmd_handle_get_ocd_voltage_threshold(const uint8_t *payload, uint16_t len, cmd_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* CMD_HANDLER_H */

