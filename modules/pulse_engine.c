/**
 ******************************************************************************
 * @file      pulse_engine.c
 * @author    ZJY
 * @version   V1.0
 * @date      2024-11-06
 * @brief     脉冲引擎模块实现文件，负责脉冲序列的配置、参数管理和输出控制
 *
 * @details   本模块支持两种工作模式：
 *            1. 正常模式：由控制命令启动输出
 *            2. 心电同步模式：等待心电R波触发后延时输出
 *
 * @copyright Copyright (C) 2024 Hangzhou Dinova EP Technology Co.,Ltd
 *            All rights reserved.
 ******************************************************************************
 */
/*------------------------------ include -------------------------------------*/
#include "pulse_engine.h"
#include "bsp_conf.h"
#include "bsp_hrtim.h"
#include "bsp_tim.h"
#include "board.h"
#include "bsp_sram.h"
#include <string.h>

/*------------------------------ Macro definition ----------------------------*/
#define HRTIM_COMPARE_DISABLED_VALUE    (0xFFFBU)   /**< HRTIM比较禁用值 */
#define TRAIN_GAP_MIN_TICKS             (332U)      /**< 最小串间隙tick值 */
#define TIMER_CLOCK_FREQUENCY           (170000000UL) /**< 定时器时钟频率(Hz) */
#define MS_TO_NS_MULTIPLIER             (1000000UL) /**< 毫秒转纳秒乘数 */

#define TIM_HZ   100000u
#define TICK_MS  100u

/*------------------------------ typedef definition --------------------------*/

/**
 * @brief 脉冲参数管理结构体
 */
typedef struct {
    pulse_params_t  seq_buf[GROUP_NUM_MAX]; /**< 脉冲序列参数缓冲区 */
    uint8_t         group_num;              /**< 已配置的脉冲群个数 */
    uint8_t         expected_num_of_group;  /**< 期望接收的脉冲群总数 */
    uint8_t         expected_group_num;     /**< 期望接收的脉冲群编号 */
    uint16_t        group_gap_ms;           /**< 群间隙时间(ms) */
} pulse_param_mgmt_t;

/**
 * @brief 脉冲控制结构体
 */
typedef struct {
    pulse_mode_t            mode;               /**< 脉冲输出模式 */
    volatile pulse_status_t status;             /**< 脉冲输出状态 */
    volatile uint16_t       output_count;       /**< 脉冲输出次数计数器 */
    volatile uint16_t       group_cnt;          /**< 当前输出的脉冲群计数器 */
    bool                    hardware_configured;/**< 硬件参数配置标志位 */
} pulse_control_t;

/**
 * @brief 硬件配置结构体
 */
typedef struct {
    burst_dma_t         dma_buf[PERIOD_PER_TRAIN_NUM_MAX]; /**< HRTIM硬件参数缓冲区 */
    volatile uint32_t   train_gap_buf[TRAIN_PER_GROUP_NUM_MAX]; /**< 串间隙硬件参数缓冲区 */
} pulse_hw_config_t;

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
/** @brief 脉冲参数管理 */
static pulse_param_mgmt_t g_params = {0};

/** @brief 脉冲控制 */
static pulse_control_t g_control = {
    .mode = PULSE_MODE_NORMAL,
    .status = PULSE_STATUS_IDLE,
    .output_count = 0U,
    .group_cnt = 0U,
    .hardware_configured = false
};

/** @brief 硬件配置（DMA缓冲区存放在外部SRAM） */
static EXTSRAM pulse_hw_config_t g_hw_config = {0};

/** @brief 心电同步状态 */
static volatile ecg_sync_state_t g_ecg_sync = {0};

/*------------------------------ function prototypes -------------------------*/
static bool param_is_valid(const pulse_params_t *param);
static void config_next_group_params(uint8_t group_idx);
static void start_pulse_output(void);
static void stop_pulse_output(void);
static int32_t pulse_out_set_hardware(void);
static int32_t pulse_out_set_group_gap(uint16_t gap_value);
static void start_pulse_internal(void);
static void go_wait_r_trigger(void);
static void config_stop_time_timer(uint32_t stop_time_ms);
static void config_delay_timer(uint32_t delay_ms);

/*------------------------------ application ---------------------------------*/

/**
 * @brief 初始化脉冲引擎模块
 *
 * @details 将所有内部状态变量和参数缓冲区重置为初始值
 *
 * @return 无
 */
void pulse_engine_init(void)
{
    g_control.mode = PULSE_MODE_NORMAL;
    g_control.status = PULSE_STATUS_IDLE;
    g_control.output_count = 0U;
    g_control.group_cnt = 0U;
    g_control.hardware_configured = false;
    
    g_params.group_num = 0U;
    g_params.expected_num_of_group = 0U;
    g_params.expected_group_num = 1U;
    g_params.group_gap_ms = 0U;
    (void)memset(g_params.seq_buf, 0, sizeof(g_params.seq_buf));
    
    /* 初始化心电同步状态结构体 */
    (void)memset((void *)&g_ecg_sync, 0, sizeof(g_ecg_sync));
    g_ecg_sync.state = SYNC_STATUS_IDLE;
}

/**
 * @brief 设置脉冲输出模式
 *
 * @param[in] mode 脉冲输出模式
 *
 * @return int32_t 返回状态
 * @retval  0 设置成功
 * @retval -1 设置失败（参数无效或正在运行中）
 *
 * @note 只能在IDLE或COMPLETED状态下设置模式
 */
int32_t pulse_engine_set_mode(pulse_mode_t mode)
{
    int32_t result = -1;
    
    if ((mode == PULSE_MODE_NORMAL) || (mode == PULSE_MODE_ECG_SYNC)) {
        if (g_control.status != PULSE_STATUS_RUNNING) {
            g_control.mode = mode;
            result = 0;
        }
    }
    
    return result;
}

/**
 * @brief 获取当前脉冲输出模式
 *
 * @param[out] mode 脉冲输出模式指针
 *
 * @return int32_t 返回状态
 * @retval  0 获取成功
 * @retval -1 获取失败（参数为空指针）
 */
int32_t pulse_engine_get_mode(pulse_mode_t* mode)
{
    int32_t result = -1;
    
    if (mode != NULL) {
        *mode = g_control.mode;
        result = 0;
    }
    
    return result;
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
int32_t pulse_engine_set_sync_param(const ecg_sync_cfg_t* config)
{
    int32_t result = -1;
    
    if (config != NULL) {
        if ((config->trigger_delay_ms <= 300U) && 
            (config->stop_delay_ms >= 200U) && (config->stop_delay_ms <= 2000U) &&
            (config->interval_R <= 30U) &&
            (config->repeat_count >= 1U) && (config->repeat_count <= 300U)) {
            if (g_control.status != PULSE_STATUS_RUNNING) {
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
int32_t pulse_engine_get_sync_param(ecg_sync_cfg_t* config)
{
    int32_t result = -1;
    
    if (config != NULL) {
        (void)memcpy(config, (const void *)&g_ecg_sync.cfg, sizeof(ecg_sync_cfg_t));
        result = 0;
    }
    
    return result;
}

/**
 * @brief 设置脉冲序列参数
 *
 * @param[in] p 脉冲序列参数指针
 *
 * @return int32_t 返回状态
 * @retval  0 设置成功
 * @retval -1 设置失败（参数无效、正在运行或缓冲区已满）
 *
 * @details 支持分批设置参数，第一个参数的group_num必须为1。
 *          如果接收到group_num=1，会清除之前的所有参数，重新开始。
 *          群编号必须按顺序连续设置（1, 2, 3, ...）。
 *
 * @note 必须在非运行状态下设置
 */
int32_t pulse_engine_set_seq_param(const pulse_params_t* p)
{
    int32_t result = -1;
    
    if (p != NULL) {
        if (param_is_valid(p)) {
            if (g_control.status != PULSE_STATUS_RUNNING) {
                /* 如果是第一个群参数（group_num=1），清除之前的参数，重新开始 */
                if (p->group_num == 1U) {
                    g_params.group_num = 0U;
                    g_params.expected_num_of_group = p->num_of_group;
                    g_params.expected_group_num = 1U;
                    g_params.group_gap_ms = p->group_gap;
                    g_control.hardware_configured = false;
                    (void)memset(g_params.seq_buf, 0, sizeof(g_params.seq_buf));
                }
                
                /* 检查群编号有效性：必须是期望的顺序编号 */
                if (p->group_num == g_params.expected_group_num) {
                    /* 保存参数 */
                    if (g_params.group_num < GROUP_NUM_MAX) {
                        (void)memcpy(&g_params.seq_buf[g_params.group_num], p, sizeof(pulse_params_t));
                        g_params.group_num++;
                        g_params.expected_group_num++;
                        if (g_params.expected_num_of_group == p->group_num) {
                            pulse_out_set_hardware();
                            g_control.hardware_configured = true;
                        }
                        result = 0;
                    }
                }
            }
        }
    }
    
    return result;
}

/**
 * @brief 获取脉冲序列参数
 *
 * @param[out] p 脉冲序列参数数组指针
 * @param[out] total_groups 总群数指针
 *
 * @return int32_t 返回状态
 * @retval  0 获取成功
 * @retval -1 获取失败（参数为空指针）
 */
int32_t pulse_engine_get_seq_param(pulse_params_t* p, uint16_t* total_groups)
{
    int32_t result = -1;
    
    if ((p != NULL) && (total_groups != NULL)) {
        (void)memcpy(p, g_params.seq_buf, sizeof(pulse_params_t) * g_params.group_num);
        *total_groups = g_params.group_num;
        result = 0;
    }
    
    return result;
}

/**
 * @brief 获取脉冲输出状态
 *
 * @param[out] report 状态上报结构体指针
 *
 * @return int32_t 返回状态
 * @retval  0 获取成功
 * @retval -1 获取失败（参数为空指针）
 */
int32_t pulse_engine_get_status(pulse_report_t* report)
{
    int32_t result = -1;
    
    if (report != NULL) {
        report->mode = g_control.mode;
        report->status = g_control.status;
        report->count = g_control.output_count;
        result = 0;
    }
    
    return result;
}

/**
 * @brief 通知脉冲输出完成
 *
 * @details 由中断或任务调用，用于更新输出状态和计数
 *
 * @return 无
 */
void pulse_engine_notify_output_complete(void)
{
    g_control.status = PULSE_STATUS_COMPLETED;
    g_control.output_count++;
}

/**
 * @brief 参数有效性检查
 *
 * @param[in] param 脉冲参数指针
 *
 * @return bool 参数是否有效
 * @retval true  参数有效
 * @retval false 参数无效
 */
static bool param_is_valid(const pulse_params_t *param)
{
    bool is_valid = true;
    
    /* 脉冲序列参数检查 */
    if ((param->num_of_group < GROUP_NUM_MIN) || (param->num_of_group > GROUP_NUM_MAX)) {
        is_valid = false;
    } else if ((param->group_gap < GROUP_GAP_MIN) || (param->group_gap > GROUP_GAP_MAX)) {
        is_valid = false;
    } else if ((param->pos_pw < PULSE_PW_MIN) || (param->pos_pw > PULSE_PW_MAX)) {
        is_valid = false;
    } else if (((param->neg_pw > 0U) && (param->neg_pw < PULSE_PW_MIN)) || 
               (param->neg_pw > PULSE_PW_MAX)) {
        is_valid = false;
    } else if ((param->pn_gap < PULSE_GAP_MIN) || (param->pn_gap > PULSE_GAP_MAX)) {
        is_valid = false;
    } else if ((param->np_gap < PULSE_GAP_MIN) || (param->np_gap > PULSE_GAP_MAX)) {
        is_valid = false;
    } else if ((param->periods_per_train < PERIOD_PER_TRAIN_NUM_MIN) || 
               (param->periods_per_train > PERIOD_PER_TRAIN_NUM_MAX)) {
        is_valid = false;
    } else if ((param->train_gap > TRAIN_GAP_MAX) || 
               ((param->train_gap > 0U) && (param->train_gap < TRAIN_GAP_MIN))) {
        is_valid = false;
    } else if ((param->train_per_group < TRAIN_PER_GROUP_NUM_MIN) || 
               (param->train_per_group > TRAIN_PER_GROUP_NUM_MAX)) {
        is_valid = false;
    } else {
#if PULSE_PARAMETERS_ARE_VARIABLE
        /* 脉冲变化参数检查 */
        if ((param->pos_pw_step > PULSE_PW_STEP_MAX) ||
            (param->neg_pw_step > PULSE_PW_STEP_MAX) ||
            (param->pn_gap_step > PULSE_GAP_STEP_MAX) ||
            (param->np_gap_step > PULSE_GAP_STEP_MAX)) {
            is_valid = false;
        } else if ((param->pos_pw_var < PULSE_PW_VAR_MIN) || (param->pos_pw_var > PULSE_PW_VAR_MAX) ||
                   (param->neg_pw_var < PULSE_PW_VAR_MIN) || (param->neg_pw_var > PULSE_PW_VAR_MAX) ||
                   (param->pn_gap_var < PULSE_GAP_VAR_MIN) || (param->pn_gap_var > PULSE_GAP_VAR_MAX) ||
                   (param->np_gap_var < PULSE_GAP_VAR_MIN) || (param->np_gap_var > PULSE_GAP_VAR_MAX)) {
            is_valid = false;
        } else {
            /* 所有检查通过 */
        }
#endif
    }
    
    return is_valid;
}

/**
 * @brief 配置群间隙定时器参数
 *
 * @param[in] gap_value 群间隙时间(ms)
 *
 * @return int32_t 返回状态
 * @retval  0 配置成功
 * @retval -1 配置失败
 */
static int32_t pulse_out_set_group_gap(uint16_t gap_value)
{
    uint16_t psc = 0U;
    uint16_t arr = 0U;
    uint64_t actual_ns = 0UL;
    
    tim_pick_psc_arr_from_ns(TIMER_CLOCK_FREQUENCY,
                             (uint64_t)gap_value * MS_TO_NS_MULTIPLIER,
                             &psc,
                             &arr,
                             &actual_ns);
    
    /* 先停止TIM6，确保配置生效 */
    __HAL_TIM_DISABLE(&htim6);
    
    __HAL_TIM_SET_PRESCALER(&htim6, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    
    return 0;
}

/**
 * @brief 配置硬件参数
 *
 * @details 根据脉冲序列参数计算并配置HRTIM硬件参数
 *
 * @return int32_t 返回状态
 * @retval  0 配置成功
 * @retval -1 配置失败
 */
static int32_t pulse_out_set_hardware(void)
{
    const pulse_params_t *param = NULL;
    uint8_t i;
    
    /* 遍历所有群参数并配置硬件 */
    for (i = 0U; i < g_params.group_num; i++)
    {
        param = &g_params.seq_buf[i];
        
        /* 脉冲周期硬件参数计算 */
        if (param->neg_pw > 0U) {
            /* 双极性脉冲配置 */
            g_hw_config.dma_buf[i].compare1 = param->np_gap * HRTIM_RESOLUTION;
            g_hw_config.dma_buf[i].compare2 = (param->np_gap + param->pos_pw) * HRTIM_RESOLUTION;
            g_hw_config.dma_buf[i].compare3 = (param->np_gap + param->pos_pw + param->pn_gap) * HRTIM_RESOLUTION;
            g_hw_config.dma_buf[i].period_A = (param->np_gap + param->pos_pw + param->pn_gap + param->neg_pw) * HRTIM_RESOLUTION;
        } else {
            /* 单极性脉冲配置 */
            g_hw_config.dma_buf[i].compare1 = param->np_gap * HRTIM_RESOLUTION;
            g_hw_config.dma_buf[i].compare2 = HRTIM_COMPARE_DISABLED_VALUE;
            g_hw_config.dma_buf[i].compare3 = HRTIM_COMPARE_DISABLED_VALUE;
            g_hw_config.dma_buf[i].period_A = (param->np_gap + param->pos_pw) * HRTIM_RESOLUTION;
        }
        
        /* 每串脉冲周期个数配置 */
        g_hw_config.dma_buf[i].period_B = g_hw_config.dma_buf[i].period_A;
        g_hw_config.dma_buf[i].repetion_A = param->periods_per_train - 1U;
        
        /* 串个数配置 */
        g_hw_config.dma_buf[i].repetion_B = (param->periods_per_train * param->train_per_group) - 1U;
        
        /* 串间隙配置 */
        if (param->train_gap == 0U) {
            g_hw_config.train_gap_buf[i] = TRAIN_GAP_MIN_TICKS;
        } else {
            g_hw_config.train_gap_buf[i] = tim_arr_from_ms(param->train_gap);
        }
    }
    
    /* 群间隙配置 */
    (void)pulse_out_set_group_gap(g_params.seq_buf[0].group_gap);
    
    return 0;
}

/**
 * @brief 启动脉冲序列输出
 *
 * @return int32_t 返回状态
 * @retval  0 启动成功
 * @retval -1 启动失败（参数不完整或正在运行中）
 *
 * @details 支持重复启动，使用相同参数多次输出。
 *          根据模式选择立即启动（正常模式）或等待触发（心电同步模式）。
 */
int32_t pulse_engine_start(void)
{
    int32_t result = -1;
    
    /* 检查参数是否完整 */
    if ((g_params.group_num == g_params.expected_num_of_group) && (g_params.group_num > 0U)) {
        /* 检查是否正在运行且硬件参数已经配置 */
        if (g_control.status != PULSE_STATUS_RUNNING && g_control.hardware_configured != false) {
            /* 根据模式启动 */
            if (g_control.mode == PULSE_MODE_NORMAL) {
                /* 正常模式：立即启动 */
                g_control.status = PULSE_STATUS_RUNNING;
                g_control.group_cnt = 0U;
                g_control.output_count = 0U;
                
                /* 确保TIM6处于停止状态并清零计数器 */
                __HAL_TIM_DISABLE(&htim6);
                __HAL_TIM_SET_COUNTER(&htim6, 0U);
                
                /* 配置第一组脉冲参数 */
                config_next_group_params(0U);
                
                /* 启动脉冲输出 */
                start_pulse_output();
                
                result = 0;
            } else if (g_control.mode == PULSE_MODE_ECG_SYNC) {
                /* 心电同步触发模式：等待心电触发 */
                g_control.status = PULSE_STATUS_RUNNING;
                g_control.group_cnt = 0U;
                
                /* 心电同步模式首次启动时重置计数 */
                if ((g_control.output_count == 0U) || (g_control.output_count >= g_ecg_sync.cfg.repeat_count)) {
                    g_control.output_count = 0U;
                }
                
                /* 启动心电R波检测模块，等待触发 */
                ECG_SYNC_Start(&g_ecg_sync.cfg);
                result = 0;
            } else {
                /* 未知模式 */
            }
        }
    }
    
    return result;
}

/**
 * @brief 停止脉冲序列输出
 *
 * @return int32_t 返回状态
 * @retval 0 停止成功
 *
 * @details 停止HRTIM定时器并清空计数器，将状态设置为空闲
 *          如果是心电同步模式，同时停止心电检测
 */
int32_t pulse_engine_stop(void)
{
    /* 停止HRTIM硬件 */
    LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2);
    LL_HRTIM_TIM_CounterDisable(HRTIM1, LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B);
    LL_HRTIM_TIM_SetCounter(HRTIM1, LL_HRTIM_TIMER_A, 0U);
    LL_HRTIM_TIM_SetCounter(HRTIM1, LL_HRTIM_TIMER_B, 0U);
    LL_HRTIM_BM_Disable(HRTIM1);
    
    /* 停止群间隙定时器 */
    __HAL_TIM_DISABLE(&htim6);
    
    /* 如果是心电同步模式，停止心电检测 */
    if (g_control.mode == PULSE_MODE_ECG_SYNC) {
        ECG_SYNC_Cancel();
    }
    
    g_control.status = PULSE_STATUS_IDLE;
    g_control.group_cnt = 0U;
    
    return 0;
}

/**
 * @brief 配置下一组脉冲参数
 *
 * @param[in] group_idx 脉冲群索引
 *
 * @return 无
 *
 * @details 将指定群的参数配置到HRTIM硬件寄存器中
 */
static inline void config_next_group_params(uint8_t group_idx)
{
    /* 脉冲周期参数配置 */
    LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, g_hw_config.dma_buf[group_idx].compare1);
    LL_HRTIM_TIM_SetCompare2(HRTIM1, LL_HRTIM_TIMER_A, g_hw_config.dma_buf[group_idx].compare2);
    LL_HRTIM_TIM_SetCompare3(HRTIM1, LL_HRTIM_TIMER_A, g_hw_config.dma_buf[group_idx].compare3);
    LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, g_hw_config.dma_buf[group_idx].period_A);

    /* 每串脉冲周期个数配置 */
    LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_A, g_hw_config.dma_buf[group_idx].repetion_A);

    /* 串个数配置 */
    LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_B, g_hw_config.dma_buf[group_idx].period_B);
    LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_B, g_hw_config.dma_buf[group_idx].repetion_B);

    /* 更新定时器配置 */
    LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_A);
    LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_B);
    
    /* 配置串间隙 */
    LL_HRTIM_BM_SetPeriod(HRTIM1, g_hw_config.train_gap_buf[group_idx]);
    LL_HRTIM_BM_SetCompare(HRTIM1, g_hw_config.train_gap_buf[group_idx]);
}

/**
 * @brief 停止脉冲输出
 *
 * @return 无
 *
 * @details 禁用HRTIM输出并停止定时器，清空计数器
 */
static inline void stop_pulse_output(void)
{
    LL_HRTIM_DisableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2);
    LL_HRTIM_TIM_CounterDisable(HRTIM1, LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B);
    LL_HRTIM_TIM_SetCounter(HRTIM1, LL_HRTIM_TIMER_A, 0U);
    LL_HRTIM_TIM_SetCounter(HRTIM1, LL_HRTIM_TIMER_B, 0U);
    LL_HRTIM_BM_Disable(HRTIM1);
}

/**
 * @brief 启动脉冲输出
 *
 * @return 无
 *
 * @details 使能HRTIM输出并启动定时器
 */
static inline void start_pulse_output(void)
{
    LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2);
    LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B);
    LL_HRTIM_BM_Enable(HRTIM1);
}

/**
 * @brief 内部启动脉冲输出（用于心电触发）
 *
 * @return 无
 *
 * @details 心电触发时调用，不进行状态检查，直接启动硬件
 * @note 调用此函数前必须确保脉冲引擎已处于RUNNING状态
 */
static inline void start_pulse_internal(void)
{
    /* 重置群计数器 */
    g_control.group_cnt = 0U;
    
    /* 配置第一组脉冲参数 */
    config_next_group_params(0U);
    
    /* 启动脉冲输出 */
    start_pulse_output();
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
 * @brief 启动心电同步触发
 *
 * @param[in] cfg 心电同步配置参数指针（可接受volatile限定符）
 *
 * @return 无
 *
 * @details 启动时只启动输入捕获中断等待R波，
 *          触发延时使用TIM6定时器实现
 */
void ECG_SYNC_Start(const volatile ecg_sync_cfg_t *cfg)
{
    if (cfg != NULL) {
        g_ecg_sync.cfg          = *cfg;
        g_ecg_sync.repeats_left = (cfg->repeat_count != 0U) ? cfg->repeat_count : 1U;
        g_ecg_sync.r_cnt_stop   = 0U;

        __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC1);
        __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
        __HAL_TIM_ENABLE(&htim4);
        
        go_wait_r_trigger();
    }
}

/**
 * @brief 取消心电同步触发
 *
 * @return 无
 *
 * @details 停止所有心电检测中断并重置状态
 */
void ECG_SYNC_Cancel(void)
{
    g_ecg_sync.state = SYNC_STATUS_IDLE;
    
    /* 停止输入捕获中断（检测R波） */
    __HAL_TIM_DISABLE(&htim4);
    
    /* 停止所有定时器 */
    __HAL_TIM_DISABLE(&htim6);
    __HAL_TIM_DISABLE(&htim7);
    __HAL_TIM_DISABLE(&htim15);
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
 * @brief HRTIM定时器B全局中断处理函数
 *
 * @details 当一组脉冲输出完成时触发，负责配置下一组参数或完成输出
 *
 * @return 无
 */
void HRTIM1_TIMB_IRQHandler(void)
{
    if(LL_HRTIM_IsActiveFlag_REP(HRTIM1, LL_HRTIM_TIMER_B))
    {
        /* 清除中断标志 */
        LL_HRTIM_ClearFlag_REP(HRTIM1, LL_HRTIM_TIMER_B);
        
        /* 停止当前脉冲输出 */
        stop_pulse_output();
        
        /* 更新群计数 */
        g_control.group_cnt++;
        
        /* 检查是否还有下一组脉冲 */
        if (g_control.group_cnt < g_params.group_num) {
            /* 配置下一组脉冲参数 */
            config_next_group_params((uint8_t)g_control.group_cnt);
            
            /* 启动群间隙计数器 */
            __HAL_TIM_SET_COUNTER(&htim6, 0U);
            __HAL_TIM_ENABLE(&htim6);
        } else {
            /* 所有群输出完成 */
            g_control.group_cnt = 0U;
            
            /* 根据模式处理完成逻辑 */
            if (g_control.mode == PULSE_MODE_ECG_SYNC) {
                /* 心电同步模式：脉冲输出完成，进入停止时间 */
                pulse_engine_notify_output_complete();
                
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
                } else {
                    /* 所有重复已完成，切换到空闲状态 */
                    g_control.status = PULSE_STATUS_IDLE;
                }
            } else {
                /* 正常模式 */
                pulse_engine_notify_output_complete();
                g_control.status = PULSE_STATUS_IDLE;
            }
        }
    }
}

/**
 * @brief 定时器周期中断回调函数
 *
 * @param[in] htim 定时器句柄指针
 *
 * @return 无
 *
 * @details 三个定时器的用途：
 *          1. TIM6：群间隙定时器（正常模式和心电同步模式都会用）
 *          2. TIM7：触发延时定时器（仅心电同步模式）
 *          3. TIM15：停止时间定时器（仅心电同步模式）
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* 群间隙定时器超时，启动下一组脉冲输出 */
        start_pulse_output();
        __HAL_TIM_DISABLE(&htim6);
    } else if (htim->Instance == TIM7) {
        /* 触发延时到达，启动脉冲输出 */
        __HAL_TIM_DISABLE(&htim7);
        g_ecg_sync.state = SYNC_STATUS_PULSING;
        start_pulse_internal();
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

