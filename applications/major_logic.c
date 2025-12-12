/**
  ******************************************************************************
  * @file        : major_logic.c
  * @brief       : 应用主逻辑协调器实现
  * @details     本文件实现了major_logic.h中定义的主逻辑协调功能。
  *              职责：
  *              - 协调各业务模块的工作
  *              - 处理模块间的事件和通知
  *              - 管理系统级状态和流程
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "major_logic.h"
#include <stdint.h>
#include <string.h>
#include "pulse_engine.h"
#include "safety.h"
#include "data_mgmt.h"
#include "cmd_handler.h"
#include "ocd.h"
#include "custom_slave.h"

#define  LOG_TAG             "major_logic"
#define  LOG_LVL             4
#include "log.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define RESET_DELAY_MS  (100U)  /**< 复位延时时间（毫秒） */

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 系统复位请求标志
 */
static volatile uint8_t g_reset_requested = 0U;

/**
 * @brief 复位请求时间戳
 */
static uint32_t g_reset_request_tick = 0U;

/**
 * @brief 上次的脉冲引擎状态（用于检测状态变化）
 */
static volatile pulse_status_t g_last_pulse_status = PULSE_STATUS_IDLE;

/* Private function prototypes -----------------------------------------------*/
static void on_pulse_complete_handler(void);
static uint32_t get_tick(void);
static void on_ocp_io_handler(void *arg);
static void on_ocp_info_callback(const ocp_info_t *info);

/* Exported variables  -------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化主逻辑协调器
 * @details 协调各模块的初始化和设置模块间的回调
 */
void major_logic_init(void)
{
    int32_t ret;

    /* 初始化数据管理模块 */
    data_mgmt_init();

    /* 初始化过流检测模块 */
    ret = ocd_init();
    if (ret != 0) {
        LOG_E("Failed to init OCD: %d", ret);
    }

    /* 启动ADC DMA循环采样 */
    ret = ocd_start();
    if (ret != 0) {
        LOG_E("Failed to start OCD: %d", ret);
    }

    /* 初始化脉冲引擎 */
    ret = pulse_engine_init();
    if (ret != 0) {
        LOG_E("Failed to initialize pulse engine: %d", ret);
    }

    /* 设置过流检测模块的过流信息回调 */
    ocd_set_comp_callback(on_ocp_info_callback);
    ocd_set_io_callback(on_ocp_io_handler);

    /* 初始化状态 */
    g_reset_requested = 0U;
    g_reset_request_tick = 0U;
    g_last_pulse_status = PULSE_STATUS_IDLE;

    LOG_D("Major logic initialized");
}

/**
 * @brief 主逻辑协调器任务
 * @details 定期调用，处理模块间的协调和状态监测
 */
void major_logic_task(void)
{
    pulse_report_t report;
    uint32_t current_tick;

    /* 处理系统复位请求 */
    if (g_reset_requested) {
        current_tick = get_tick();
        if ((current_tick - g_reset_request_tick) >= RESET_DELAY_MS) {
            /* 执行软件复位 */
            safety_perform_software_reset();
        }
    }

    /* 处理过流事件：查找峰值并上报 */
    ocd_process_ocp_event();

    /* 检查脉冲引擎状态变化，自动上报 */
    if (pulse_engine_get_status(&report) == 0) {
        if (report.status == PULSE_STATUS_IDLE &&
            g_last_pulse_status == PULSE_STATUS_RUNNING) {
            /* 状态从运行变为空闲，表示完成 */
            on_pulse_complete_handler();
        }
        /* 更新状态 */
        g_last_pulse_status = report.status;
    }
}

/**
 * @brief 请求系统复位
 * @details 设置复位请求标志，延时后执行复位
 */
void major_logic_request_reset(void)
{
    if (!g_reset_requested) {
        g_reset_requested = 1U;
        g_reset_request_tick = get_tick();
        LOG_I("System reset requested, will execute in %d ms", RESET_DELAY_MS);
    }
}

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 获取系统时间戳
 * @return 系统时间戳（毫秒）
 */
static uint32_t get_tick(void)
{
    extern uint32_t HAL_GetTick(void);
    return HAL_GetTick();
}

/**
 * @brief 脉冲完成事件处理器
 * @details 当脉冲引擎完成输出时被调用
 */
static void on_pulse_complete_handler(void)
{
    /* 通知命令处理服务上报状态 */
    cmd_handler_notify_pulse_complete();
}

/**
 * @brief 过流信息回调函数
 * @details 当电流监控模块检测到过流并找到峰值时被调用
 */
static void on_ocp_info_callback(const ocp_info_t *info)
{
    if (info == NULL) {
        return;
    }

    /* 设置过流锁定 */
    pulse_engine_ctrl_lock(1U);

    /* 上报过流信息给上位机 */
    slave_upload_ocp_info(info);
    
    LOG_I("Over-current detected: CH%d, voltage=%d mV",
          info->channel, info->voltage_mv);
}

/**
 * @brief 过流保护事件处理器
 * @details 当检测到过流时被调用
 */
static void on_ocp_io_handler(void *arg)
{
    (void)arg;
    
    /* 停止脉冲输出 */
    pulse_engine_stop();
    
    /* 设置过流锁定 */
    pulse_engine_ctrl_lock(1U);

    /* 上报过流信息给上位机 */
    
    LOG_I("Over-current detected! Stopping pulse engine...");
}

/**
  * @brief This function handles HRTIM1 fault global interrupt.
  */
void HRTIM1_FLT_IRQHandler(void)
{
    /* 检查是否是 Fault 4 (COMP1) 触发 */
    if(LL_HRTIM_IsActiveFlag_FLT4(HRTIM1))
    {
        LL_HRTIM_ClearFlag_FLT4(HRTIM1);
        
    }

    /* 检查 Fault 5 (COMP3) */
    if(LL_HRTIM_IsActiveFlag_FLT5(HRTIM1))
    {
        LL_HRTIM_ClearFlag_FLT5(HRTIM1);
    }
}
