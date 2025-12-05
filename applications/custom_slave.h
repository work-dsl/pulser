/**
  ******************************************************************************
  * @file        : custom_slave.h
  * @brief       : 自定义协议从机接口
  * @details     本文件定义了自定义协议从机的接口函数。
  *              从机负责接收主机命令、处理命令并回送响应。
  * @attention   使用前需要先调用 slave_proto_init() 进行初始化
  ******************************************************************************
  */

#ifndef CUSTOM_SLAVE_H
#define CUSTOM_SLAVE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ocd.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 从机握手状态枚举
 */
typedef enum {
    SLAVE_STATE_WAIT_HANDSHAKE = 0,  /**< 等待握手状态 */
    SLAVE_STATE_ACTIVE               /**< 活动状态（已握手） */
} slave_state_t;

/* Exported constants --------------------------------------------------------*/

/**
 * @defgroup 地址定义
 * @{
 */
#define PRODUCT_ADDR                        (0x03U) /**< 产品地址（用于区分不同的系列产品） */
#define MODULE_ADDR                         (0x02U) /**< 模块地址（用于区分同一个产品中的不同模块） */
/**
 * @}
 */

#define HANDSHAKE_INTERVAL_MS               (1000U) /**< 握手间隔：1秒（1Hz） */

/**
 * @defgroup 脉冲序列参数相关定义
 * @{
 */
#define PULSE_PARAM_SIZE                    (sizeof(pulse_params_t))  /**< 脉冲参数结构体大小 */
#define ECG_SYNC_PARAM_SIZE                 (sizeof(ecg_sync_cfg_t))  /**< 心电同步参数结构体大小 */
/**
 * @}
 */

/**
 * @defgroup 从机专有命令
 * @{
 */
#define CMD_CTRL_PULSE_ENGINE_START         (0x30)  /**< 控制脉冲序列输出 */
#define CMD_GET_PULSE_ENGINE_STATUS         (0x31)  /**< 获取脉冲序列输出状态 */
#define CMD_SET_PULSE_ENGINE_TRIGGER_MODE   (0x32)  /**< 设置脉冲序列输出模式 */
#define CMD_GET_PULSE_ENGINE_TRIGGER_MODE   (0x33)  /**< 获取脉冲序列输出模式 */
#define CMD_SET_PULSE_ENGINE_PARAMETERS     (0x34)  /**< 设置脉冲序列参数 */
#define CMD_GET_PULSE_ENGINE_PARAMETERS     (0x35)  /**< 获取脉冲序列参数 */
#define CMD_SET_ECG_SYNC_TRIGGER_PARAMETERS (0x36)  /**< 设置心电同步触发参数 */
#define CMD_GET_ECG_SYNC_TRIGGER_PARAMETERS (0x37)  /**< 获取心电同步触发参数 */
#define CMD_SET_OCD_VOLTAGE_THRESHOLD       (0x38)  /**< 设置过流检测的电压阈值 */
#define CMD_GET_OCD_VOLTAGE_THRESHOLD       (0x39)  /**< 获取过流检测的电压阈值 */
#define CMD_PULSE_ENGINE_STATUS_UPLOAD      (0x3A)  /**< 脉冲序列输出完成状态上报 */
#define CMD_OVER_CURRENT_STATUS_UPLOAD      (0x3B)  /**< 过流状态信息上报 */
#define CMD_GET_OCP_PIN_STATUS              (0x3C)  /**< 读取硬件IO过流保护引脚状态 */
#define CMD_RESET_OCP_HARDWARE              (0x3D)  /**< 复位硬件过流保护 */
/**
 * @}
 */


/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

int  slave_proto_init(void);
void slave_proto_task(void);
void slave_process_frame(const uint8_t *frame, uint16_t len);
slave_state_t slave_get_state(void);
void slave_upload_ocp_info(const ocp_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_SLAVE_H */