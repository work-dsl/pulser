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
#include "current_monitor.h"

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
 * @defgroup 从机设备地址定义
 * @{
 */
#define SLAVE_DEVICE_ADDR                   (0x03U) /**< 本机设备地址 */
#define SLAVE_MODULE_ADDR                   (0x02U) /**< 本机模块地址 */
/**
 * @}
 */

/**
* @defgroup 主机->从机最大帧长度限制
 * @{
 */
#define HOST_TO_SLAVE_MAX_LEN               (64)    /**< 主机->从机最大帧长度 */
/**
 * @}
 */

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
#define CMD_SET_PULSE_ENGINE_TRIGGER_MODE   (0x32)  /**< 设置脉冲序列输出模式：正常模式或心电同步触发 */
#define CMD_GET_PULSE_ENGINE_TRIGGER_MODE   (0x33)  /**< 获取脉冲序列输出模式 */
#define CMD_SET_PULSE_ENGINE_PARAMETERS     (0x34)  /**< 设置脉冲序列参数 */
#define CMD_GET_PULSE_ENGINE_PARAMETERS     (0x35)  /**< 获取脉冲序列参数 */
#define CMD_SET_ECG_SYNC_TRIGGER_PARAMETERS (0x36)  /**< 设置心电同步触发参数 */
#define CMD_GET_ECG_SYNC_TRIGGER_PARAMETERS (0x37)  /**< 获取心电同步触发参数 */
#define CMD_SET_OCD_VOLTAGE_THRESHOLD       (0x38)  /**< 设置过流检测的电压阈值 */
#define CMD_GET_OCD_VOLTAGE_THRESHOLD       (0x39)  /**< 获取过流检测的电压阈值 */
#define CMD_PULSE_ENGINE_STATUS_UPLOAD      (0x3A)  /**< 脉冲序列输出状态上报 */
/**
 * @}
 */


/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化从机协议解析器
 * @retval 0 成功
 * @retval <0 失败（错误码）
 * @details 初始化串口设备、协议解析器和相关回调函数
 */
int slave_proto_init(void);

/**
 * @brief 轮询从机协议解析器
 * @details 从串口FIFO中读取数据并送入协议解析器处理。
 *          建议在主循环或任务中定期调用此函数。
 */
void slave_proto_task(void);

/**
 * @brief 获取从机当前状态
 * @return 当前从机状态
 */
slave_state_t slave_get_state(void);

/**
 * @brief 重置从机到等待握手状态
 * @details 用于系统复位或重新初始化
 */
void slave_reset_to_handshake(void);

/**
 * @brief 上报过流信息给上位机
 * @param info 过流信息指针
 */
void slave_upload_ocp_info(const ocp_info_t *info);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_SLAVE_H */