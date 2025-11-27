/**
 ******************************************************************************
 * @copyright Copyright (C) 2024 Hangzhou Dinova EP Technology Co.,Ltd
 *            All rights reserved.
 * @file      ecg_sync.h
 * @author    ZJY
 * @version   V1.0
 * @date      2024-11-06
 * @brief     心电同步触发模块头文件，负责R波检测和触发时序管理
 *
 * @details   本模块负责：
 *            1. R波检测（通过TIM4输入捕获）
 *            2. 触发延时管理（TIM7定时器）
 *            3. 停止时间管理（TIM15定时器）
 *            4. 触发状态机管理
 *
 ******************************************************************************
 */

#ifndef ECG_SYNC_H
#define ECG_SYNC_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------ include -------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "board.h"

/*------------------------------ Macro definition ----------------------------*/
#define TIMER_CLOCK_FREQUENCY           (170000000UL) /**< 定时器时钟频率(Hz) */
#define MS_TO_NS_MULTIPLIER             (1000000UL)   /**< 毫秒转纳秒乘数 */

/*------------------------------ typedef definition --------------------------*/

/**
 * @brief 心电同步状态枚举
 */
typedef enum { 
    SYNC_STATUS_IDLE = 0,          /**< 空闲状态 */
    SYNC_STATUS_WAIT_R_TRIGGER,    /**< 等待触发R波 */
    SYNC_STATUS_WAIT_DELAY,        /**< 等待延时 */
    SYNC_STATUS_PULSING,            /**< 脉冲输出中 */
    SYNC_STATUS_STOP_TIME,          /**< 停止时间（计数R波） */
    SYNC_STATUS_WAIT_INTERVAL       /**< 等待R波间隔满足 */
} sync_state_t;

/**
 * @brief 心电同步触发参数配置结构体
 * @note 工作流程：R波触发 → 延时D → 脉冲序列输出 → 停止时间 → 间隔N个R波 → 重复
 */
typedef struct {
    uint16_t trigger_delay_ms;      /**< 触发输出延时，单位：ms，范围：0-300 */
    uint16_t stop_delay_ms;         /**< 停止时间，单位：ms，范围：200-2000 */
    uint16_t interval_R;            /**< R波间隔个数，范围：0-30 */
    uint16_t repeat_count;          /**< 连续触发次数，范围：1-300 */
} ecg_sync_cfg_t;

/**
 * @brief 心电触发回调函数类型
 * @details 当检测到R波并延时到达时，调用此回调通知脉冲引擎启动输出
 */
typedef void (*ecg_trigger_cb_t)(void);

/**
 * @brief 脉冲输出完成回调函数类型
 * @details 当脉冲输出完成时，调用此回调通知心电同步模块
 */
typedef void (*pulse_complete_cb_t)(void);

/*------------------------------ function declarations -----------------------*/

/**
 * @brief 初始化心电同步模块
 *
 * @return 无
 */
void ecg_sync_init(void);

/**
 * @brief 设置心电触发回调函数
 *
 * @param[in] callback 触发回调函数指针，NULL表示清除回调
 *
 * @return 无
 */
void ecg_sync_set_trigger_callback(ecg_trigger_cb_t callback);

/**
 * @brief 设置脉冲输出完成回调函数
 *
 * @param[in] callback 完成回调函数指针，NULL表示清除回调
 *
 * @return 无
 */
void ecg_sync_set_pulse_complete_callback(pulse_complete_cb_t callback);

/**
 * @brief 设置心电同步触发参数
 *
 * @param[in] config 心电同步触发参数指针
 *
 * @return int32_t 返回状态
 * @retval  0 设置成功
 * @retval -1 设置失败（参数无效或正在运行中）
 *
 * @note 必须在非运行状态下设置
 */
int32_t ecg_sync_set_param(const ecg_sync_cfg_t* config);

/**
 * @brief 获取心电同步触发参数
 *
 * @param[out] config 心电同步触发参数指针
 *
 * @return int32_t 返回状态
 * @retval  0 获取成功
 * @retval -1 获取失败（参数为空指针）
 */
int32_t ecg_sync_get_param(ecg_sync_cfg_t* config);

/**
 * @brief 启动心电同步触发
 *
 * @return int32_t 返回状态
 * @retval  0 启动成功
 * @retval -1 启动失败（参数未配置或回调未设置）
 */
int32_t ecg_sync_start(void);

/**
 * @brief 取消心电同步触发
 *
 * @return 无
 *
 * @details 停止所有心电检测中断并重置状态
 */
void ecg_sync_cancel(void);

/**
 * @brief 获取当前心电同步状态
 *
 * @return sync_state_t 当前同步状态
 */
sync_state_t ecg_sync_get_state(void);

/**
 * @brief 通知脉冲输出完成
 *
 * @details 由脉冲引擎模块调用，通知心电同步模块脉冲输出已完成
 *
 * @return 无
 */
void ecg_sync_notify_pulse_complete(void);

/**
 * @brief 处理TIM7和TIM15的周期中断
 *
 * @param[in] htim 定时器句柄指针
 *
 * @return 无
 *
 * @note 此函数由 pulse_engine.c 中的 HAL_TIM_PeriodElapsedCallback 调用
 */
void ecg_sync_handle_timer_period_elapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* ECG_SYNC_H */

/******************************* End Of File **********************************/

