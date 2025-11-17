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

void cmd_handler_init(void);
void cmd_handler_set_response_callback(cmd_response_cb_t cb);
void cmd_handler_process(uint8_t cmd, const uint8_t *payload, uint16_t len);
void cmd_handler_notify_pulse_complete(void);

#ifdef __cplusplus
}
#endif

#endif /* CMD_HANDLER_H */

