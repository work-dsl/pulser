/**
  ******************************************************************************
  * @file        : current_monitor.h
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 20xx-xx-xx
  * @brief       : 电流监控模块接口
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.初始版本
  ******************************************************************************
  */
#ifndef __CURRENT_MONITOR_H__
#define __CURRENT_MONITOR_H__

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 过流检测通道枚举
 */
typedef enum {
    CURRENT_MONITOR_CH1 = 0,  /**< ADC1通道（对应COMP3） */
    CURRENT_MONITOR_CH2 = 1   /**< ADC2通道（对应COMP1） */
} current_monitor_ch_t;

/**
 * @brief 过流信息结构体
 */
typedef struct {
    current_monitor_ch_t channel; /**< 检测通道 */
    uint16_t peak_value;          /**< 峰值ADC值 */
    uint16_t trigger_position;    /**< 触发点在缓冲区中的位置 */
    uint32_t timestamp;           /**< 触发时间戳（毫秒） */
} ocp_info_t;

/**
 * @brief 过流事件回调函数类型
 */
typedef void (*ocp_info_cb_t)(const ocp_info_t *info);

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化电流监控模块
 * @note 使用内部定义的缓冲区，无需外部传入
 * @retval 0 成功
 * @retval <0 失败
 */
int current_monitor_init(void);

/**
 * @brief 启动ADC DMA循环采样
 * @retval 0 成功
 * @retval <0 失败
 */
int current_monitor_start(void);

/**
 * @brief 停止ADC DMA循环采样
 */
void current_monitor_stop(void);

/**
 * @brief 设置过流事件回调函数
 * @param callback 回调函数指针
 */
void current_monitor_set_ocp_callback(ocp_info_cb_t callback);

/**
 * @brief 处理过流事件（在比较器中断中调用）
 * @param channel 触发通道
 */
void current_monitor_handle_ocp_trigger(current_monitor_ch_t channel);

/**
 * @brief 查找峰值并上报过流信息（在任务中调用）
 * @details 从触发点附近的数据中查找峰值，并调用回调函数上报
 */
void current_monitor_process_ocp_event(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __CURRENT_MONITOR_H__ */


