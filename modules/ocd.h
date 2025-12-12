/**
  ******************************************************************************
  * @file        : ocd.h
  * @author      : ZJY
  * @version     : V1.0
  * @date        : 20xx-xx-xx
  * @brief       : 过流检测模块接口
  * @attention   : None
  ******************************************************************************
  * @history     :
  *         V1.0 : 1.初始版本
  ******************************************************************************
  */
#ifndef __OCD_H__
#define __OCD_H__

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
    OCD_POSITIVE = 0,  /**< 正通道（对应COMP3） */
    OCD_NEGTIVE  = 1   /**< 负通道（对应COMP1） */
} ocd_ch_t;

/**
 * @brief 过流信息结构体
 */
typedef struct {
    ocd_ch_t channel;    /**< 通道 */
    uint16_t voltage_mv; /**< 过流电压值（毫伏） */
} ocp_info_t;

/**
 * @brief 过流事件回调函数类型（用于COMP+DAC检测方式）
 */
typedef void (*ocp_info_cb_t)(const ocp_info_t *info);

/**
 * @brief 过流保护事件回调函数类型（用于IO引脚检测方式）
 */
typedef void (*ocp_io_cb_t)(void *args);

/* Exported constants --------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

int     ocd_init(void);
void    ocd_set_io_callback(ocp_io_cb_t callback);
void    ocd_set_comp_callback(ocp_info_cb_t callback);
int     ocd_set_threshold(ocd_ch_t channel, uint16_t voltage_mv);
int     ocd_get_threshold(ocd_ch_t channel, uint16_t *voltage_mv);
int     ocd_start(void);
void    ocd_stop(void);
void    ocd_handle_ocp_trigger(ocd_ch_t channel);
void    ocd_process_ocp_event(void);
uint8_t ocd_get_ocp_pin_status(void);
int     ocd_reset(void);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __OCD_H__ */


