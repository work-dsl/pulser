/**
 ******************************************************************************
 * @copyright Copyright (C) 2024 Hangzhou Dinova EP Technology Co.,Ltd
 *            All rights reserved.
 * @file      ecg_sync.c
 * @author    ZJY
 * @version   V1.0
 * @date      2024-11-06
 * @brief     心电同步触发模块实现文件，负责R波检测和触发时序管理
 *
 * @details   本模块负责：
 *            1. R波检测（通过TIM4输入捕获）
 *            2. 触发延时管理（TIM7定时器）
 *            3. 停止时间管理（TIM15定时器）
 *            4. 触发状态机管理
 *
 ******************************************************************************
 */
/*------------------------------ include -------------------------------------*/
#include "ecg_sync.h"
#include "bsp_conf.h"
#include "bsp_tim.h"
#include "board.h"
#include <string.h>

/*------------------------------ Macro definition ----------------------------*/

/*------------------------------ typedef definition --------------------------*/

/**
 * @brief 心电同步状态结构体
 * @note 所有成员在中断和主程序间共享，必须使用volatile
 */
typedef struct {
    ecg_sync_cfg_t  cfg;              /**< 心电同步配置参数（中断中读取） */
    sync_state_t    state;            /**< 当前同步状态 */
    uint16_t        r_cnt_stop;       /**< 停止时间内计到的R波个数 */
    uint16_t        repeats_left;     /**< 剩余重复次数 */
} ecg_sync_state_t;

/*------------------------------ variables prototypes ------------------------*/
/** @brief 心电同步状态 */
static volatile ecg_sync_state_t g_ecg_sync = {0};

/** @brief 心电触发回调函数指针 */
static ecg_trigger_cb_t g_trigger_callback = NULL;

/** @brief 脉冲输出完成回调函数指针 */
static pulse_complete_cb_t g_pulse_complete_callback = NULL;

/*------------------------------ function prototypes -------------------------*/
static void go_wait_r_trigger(void);
static void config_stop_time_timer(uint32_t stop_time_ms);
static void config_delay_timer(uint32_t delay_ms);

/*------------------------------ application ---------------------------------*/

/**
 * @brief 初始化心电同步模块
 *
 * @return 无
 */
void ecg_sync_init(void)
{
    (void)memset((void *)&g_ecg_sync, 0, sizeof(g_ecg_sync));
    g_ecg_sync.state = SYNC_STATUS_IDLE;
    g_trigger_callback = NULL;
    g_pulse_complete_callback = NULL;
}

/**
 * @brief 设置心电触发回调函数
 *
 * @param[in] callback 触发回调函数指针，NULL表示清除回调
 *
 * @return 无
 */
void ecg_sync_set_trigger_callback(ecg_trigger_cb_t callback)
{
    g_trigger_callback = callback;
}

/**
 * @brief 设置脉冲输出完成回调函数
 *
 * @param[in] callback 完成回调函数指针，NULL表示清除回调
 *
 * @return 无
 */
void ecg_sync_set_pulse_complete_callback(pulse_complete_cb_t callback)
{
    g_pulse_complete_callback = callback;
}

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
int32_t ecg_sync_set_param(const ecg_sync_cfg_t* config)
{
    int32_t result = -1;
    
    if (config != NULL) {
        if ((config->trigger_delay_ms <= 300U) && 
            (config->stop_delay_ms >= 200U) && (config->stop_delay_ms <= 2000U) &&
            (config->interval_R <= 30U) &&
            (config->repeat_count >= 1U) && (config->repeat_count <= 300U)) {
            if (g_ecg_sync.state == SYNC_STATUS_IDLE) {
                (void)memcpy((void *)&g_ecg_sync.cfg, config, sizeof(ecg_sync_cfg_t));
                
                /* 配置触发延时定时器（TIM7） */
                config_delay_timer(config->trigger_delay_ms);
                
                /* 配置停止时间定时器（TIM15） */
                config_stop_time_timer(config->stop_delay_ms);
                
                result = 0;
            }
        }
    }
    
    return result;
}

/**
 * @brief 获取心电同步触发参数
 *
 * @param[out] config 心电同步触发参数指针
 *
 * @return int32_t 返回状态
 * @retval  0 获取成功
 * @retval -1 获取失败（参数为空指针）
 */
int32_t ecg_sync_get_param(ecg_sync_cfg_t* config)
{
    int32_t result = -1;
    
    if (config != NULL) {
        (void)memcpy(config, (const void *)&g_ecg_sync.cfg, sizeof(ecg_sync_cfg_t));
        result = 0;
    }
    
    return result;
}

/**
 * @brief 启动心电同步触发
 *
 * @return int32_t 返回状态
 * @retval  0 启动成功
 * @retval -1 启动失败（参数未配置或回调未设置）
 */
int32_t ecg_sync_start(void)
{
    int32_t result = -1;
    
    if (g_trigger_callback != NULL) {
        if (g_ecg_sync.state == SYNC_STATUS_IDLE) {
            g_ecg_sync.repeats_left = (g_ecg_sync.cfg.repeat_count != 0U) ? g_ecg_sync.cfg.repeat_count : 1U;
            g_ecg_sync.r_cnt_stop = 0U;
            
            __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC1);
            __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
            __HAL_TIM_ENABLE(&htim4);
            
            go_wait_r_trigger();
            result = 0;
        }
    }
    
    return result;
}

/**
 * @brief 取消心电同步触发
 *
 * @return 无
 *
 * @details 停止所有心电检测中断并重置状态
 */
void ecg_sync_cancel(void)
{
    g_ecg_sync.state = SYNC_STATUS_IDLE;
    
    /* 停止输入捕获中断（检测R波） */
    __HAL_TIM_DISABLE(&htim4);
    
    /* 停止所有定时器 */
    __HAL_TIM_DISABLE(&htim7);
    __HAL_TIM_DISABLE(&htim15);
}

/**
 * @brief 获取当前心电同步状态
 *
 * @return sync_state_t 当前同步状态
 */
sync_state_t ecg_sync_get_state(void)
{
    return g_ecg_sync.state;
}

/**
 * @brief 通知脉冲输出完成
 *
 * @details 由脉冲引擎模块调用，通知心电同步模块脉冲输出已完成
 *
 * @return 无
 */
void ecg_sync_notify_pulse_complete(void)
{
    if (g_ecg_sync.state == SYNC_STATUS_PULSING) {
        /* 进入停止时间，重置R波计数 */
        g_ecg_sync.state = SYNC_STATUS_STOP_TIME;
        g_ecg_sync.r_cnt_stop = 0U;
        
        /* 重新启动输入捕获中断（需要在停止时间内计数R波） */
        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC1);
        __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
        __HAL_TIM_ENABLE(&htim4);
        
        /* 启动TIM15停止时间定时器（已在设置参数时配置好） */
        __HAL_TIM_SET_COUNTER(&htim15, 0U);
        __HAL_TIM_ENABLE(&htim15);
    }
}

/**
 * @brief 进入等待触发R波状态
 *
 * @return 无
 */
static inline void go_wait_r_trigger(void)
{
    g_ecg_sync.state = SYNC_STATUS_WAIT_R_TRIGGER;
    g_ecg_sync.r_cnt_stop = 0U;
}

/**
 * @brief 配置停止时间定时器（TIM15）
 *
 * @param[in] stop_time_ms 停止时间(ms)
 *
 * @return 无
 */
static void config_stop_time_timer(uint32_t stop_time_ms)
{
    uint16_t psc = 0U;
    uint16_t arr = 0U;
    uint64_t actual_ns = 0UL;
    
    tim_pick_psc_arr_from_ns(TIMER_CLOCK_FREQUENCY,
                             stop_time_ms * MS_TO_NS_MULTIPLIER,
                             &psc,
                             &arr,
                             &actual_ns);
    
    __HAL_TIM_SET_PRESCALER(&htim15, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim15, arr);
    __HAL_TIM_SET_COUNTER(&htim15, 0U);
}

/**
 * @brief 配置触发延时定时器（TIM7）
 *
 * @param[in] delay_ms 触发延时(ms)
 *
 * @return 无
 */
static void config_delay_timer(uint32_t delay_ms)
{
    uint16_t psc = 0U;
    uint16_t arr = 0U;
    uint64_t actual_ns = 0UL;
    
    tim_pick_psc_arr_from_ns(TIMER_CLOCK_FREQUENCY,
                             delay_ms * MS_TO_NS_MULTIPLIER,
                             &psc,
                             &arr,
                             &actual_ns);
    
    __HAL_TIM_SET_PRESCALER(&htim7, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim7, arr);
    __HAL_TIM_SET_COUNTER(&htim7, 0U);
}

/**
 * @brief HAL回调函数：捕获到R波
 *
 * @param[in] htim 定时器句柄指针
 *
 * @return 无
 *
 * @details 根据当前状态处理R波事件：
 *          1. WAIT_R_TRIGGER: 启动延时定时器
 *          2. STOP_TIME: 计数R波
 *          3. WAIT_INTERVAL: 检查间隔条件，满足则启动下一轮
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) {
        return;
    }
    
    switch (g_ecg_sync.state)
    {
        case SYNC_STATUS_WAIT_R_TRIGGER:
            /* 检测到触发R波，启动延时 */
            g_ecg_sync.state = SYNC_STATUS_WAIT_DELAY;
            
            /* 关闭输入捕获中断（延时和脉冲输出期间不需要检测R波） */
            __HAL_TIM_DISABLE(&htim4);
            
            /* 启动TIM7触发延时定时器（已在设置参数时配置好） */
            __HAL_TIM_SET_COUNTER(&htim7, 0U);
            __HAL_TIM_ENABLE(&htim7);
            break;
            
        case SYNC_STATUS_STOP_TIME:
            /* 停止时间内，计数R波 */
            g_ecg_sync.r_cnt_stop++;
            break;
            
        case SYNC_STATUS_WAIT_INTERVAL:
            /* 继续计数R波 */
            g_ecg_sync.r_cnt_stop++;
            
            /* 检查间隔条件是否满足 */
            if (g_ecg_sync.r_cnt_stop >= g_ecg_sync.cfg.interval_R) {
                /* 条件满足，检查是否还有重复次数 */
                if (g_ecg_sync.repeats_left > 0U) {
                    g_ecg_sync.repeats_left--;
                    
                    if (g_ecg_sync.repeats_left > 0U) {
                        /* 还有重复次数，启动下一轮 */
                        g_ecg_sync.state = SYNC_STATUS_WAIT_DELAY;
                        
                        /* 关闭输入捕获中断（延时和脉冲输出期间不需要检测R波） */
                        __HAL_TIM_DISABLE(&htim4);
                        
                        /* 启动TIM7触发延时定时器（已在设置参数时配置好） */
                        __HAL_TIM_SET_COUNTER(&htim7, 0U);
                        __HAL_TIM_ENABLE(&htim7);
                    } else {
                        /* 所有重复完成，设置为空闲状态 */
                        g_ecg_sync.state = SYNC_STATUS_IDLE;
                    }
                }
            }
            break;
            
        default:
            /* 其他状态，忽略R波 */
            break;
    }
}

/**
 * @brief 处理TIM7和TIM15的周期中断
 *
 * @param[in] htim 定时器句柄指针
 *
 * @return 无
 *
 * @details 两个定时器的用途：
 *          1. TIM7：触发延时定时器
 *          2. TIM15：停止时间定时器
 *          
 * @note 此函数由 pulse_engine.c 中的 HAL_TIM_PeriodElapsedCallback 调用
 */
void ecg_sync_handle_timer_period_elapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        /* 触发延时到达，通知脉冲引擎启动输出 */
        __HAL_TIM_DISABLE(&htim7);
        g_ecg_sync.state = SYNC_STATUS_PULSING;
        
        if (g_trigger_callback != NULL) {
            g_trigger_callback();
        }
    } else if (htim->Instance == TIM15) {
        /* 停止时间结束，进入等待间隔状态 */
        __HAL_TIM_DISABLE(&htim15);
        g_ecg_sync.state = SYNC_STATUS_WAIT_INTERVAL;
        
        /* 立即检查间隔条件是否已满足 */
        if (g_ecg_sync.r_cnt_stop >= g_ecg_sync.cfg.interval_R) {
            /* 条件已满足，等待下一个R波触发 */
            go_wait_r_trigger();
        }
        /* 否则继续等待R波，在IC回调中检查条件 */
    }
}

/******************************* End Of File **********************************/

