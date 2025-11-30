/**
 ******************************************************************************
 * @copyright Copyright (C) 2024 Hangzhou Dinova EP Technology Co.,Ltd
 *            All rights reserved.
 * @file      pulse_engine.h
 * @author    ZJY
 * @version   V1.0
 * @date      2024-11-06
 * @brief     脉冲引擎模块头文件，提供脉冲序列参数配置和输出控制接口
 *
 ******************************************************************************
 */

#ifndef PULSE_ENGINE_H
#define PULSE_ENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------ include -------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "ecg_sync.h"

/*------------------------------ Macro definition ----------------------------*/
/* 功能配置 */
#define PULSE_PARAMETERS_ARE_VARIABLE   (0)    /* 是否支持脉冲参数动态变化 */

/* 脉冲宽度限制 */
#define PULSE_PW_MIN                    (200)   /* 最小脉冲宽度 (ns) */
#define PULSE_PW_MAX                    (10000) /* 最大脉冲宽度 (ns) */

/* 脉冲间隙限制 */
#define PULSE_GAP_MIN                   (200)   /* 最小脉冲间隙 (ns) */
#define PULSE_GAP_MAX                   (30000) /* 最大脉冲间隙 (ns) */

/* 脉冲宽度变化参数限制 */
#define PULSE_PW_STEP_MAX               (10)    /* 最大变化步进数 */
#define PULSE_PW_VAR_MIN                (-3000) /* 最小变化量 (ns) */
#define PULSE_PW_VAR_MAX                (3000)  /* 最大变化量 (ns) */

/* 脉冲间隙变化参数限制 */
#define PULSE_GAP_STEP_MAX              (40)    /* 最大变化步进数 */
#define PULSE_GAP_VAR_MIN               (-10000)/* 最小变化量 (ns) */
#define PULSE_GAP_VAR_MAX               (10000) /* 最大变化量 (ns) */

/* 脉冲周期数限制 */
#define PERIOD_PER_TRAIN_NUM_MIN        (1)     /* 每串最小脉冲周期数 */
#define PERIOD_PER_TRAIN_NUM_MAX        (300)   /* 每串最大脉冲周期数 */

/* 脉冲串间隙限制 */
#define TRAIN_GAP_MIN                   (1)     /* 最小串间隙 (ms) */
#define TRAIN_GAP_MAX                   (100)   /* 最大串间隙 (ms) */

/* 每群脉冲串数限制 */
#define TRAIN_PER_GROUP_NUM_MIN         (1)     /* 每群最小脉冲串数 */
#define TRAIN_PER_GROUP_NUM_MAX         (300)   /* 每群最大脉冲串数 */

/* 脉冲群间隙限制 */
#define GROUP_GAP_MIN                   (50)    /* 最小群间隙 (ms) */
#define GROUP_GAP_MAX                   (10000) /* 最大群间隙 (ms) */

/* 脉冲群数限制 */
#define GROUP_NUM_MIN                   (1)     /* 最小脉冲群数 */
#define GROUP_NUM_MAX                   (20)    /* 最大脉冲群数 */

/* 硬件相关参数 */
#define TICKS_PER_0P1US                 (48UL)  /* 每0.1us的计数值 */
#define ARR_MAX                         (65520UL)/* 16位计数器最大值 */

/*------------------------------ typedef definition --------------------------*/

/**
 * @brief 脉冲序列参数结构体
 */
typedef struct pulse_sequence {
    uint8_t  num_of_group;          /**< 群个数，范围：1-20 */
    uint8_t  group_num;             /**< 群编号，范围：1-20 */
    uint16_t group_gap;             /**< 群间隙，范围：50-10000ms */
    uint16_t train_per_group;       /**< 每群脉冲串数量，范围：1-300 */
    uint16_t train_gap;             /**< 脉冲串间隙，范围：1-100ms */
    uint16_t periods_per_train;     /**< 每串脉冲周期数，范围：1-300 */
    uint16_t np_gap;                /**< 负-正脉冲间隙，单位：ns */
    uint16_t pos_pw;                /**< 正脉宽，单位：ns */
    uint16_t pn_gap;                /**< 正-负脉冲间隙，单位：ns */
    uint16_t neg_pw;                /**< 负脉宽，单位：ns，0表示单极性 */
#if PULSE_PARAMETERS_ARE_VARIABLE
    uint16_t pn_gap_step;           /**< 正-负脉冲间隙变化步进 */
    int16_t  pn_gap_var;            /**< 正-负脉冲间隙变化值，单位：ns */
    uint16_t pos_pw_step;           /**< 正脉宽变化步进 */
    int16_t  pos_pw_var;            /**< 正脉宽变化值，单位：ns */
    uint16_t np_gap_step;           /**< 负-正脉冲间隙变化步进 */
    int16_t  np_gap_var;            /**< 负-正脉冲间隙变化值，单位：ns */
    uint16_t neg_pw_step;           /**< 负脉宽变化步进 */
    int16_t  neg_pw_var;            /**< 负脉宽变化值，单位：ns */
#endif
} pulse_params_t;

/**
 * @brief HRTIM硬件配置参数结构体
 */
typedef struct
{
    uint16_t period_A;              /**< 定时器A周期值 */
    uint16_t repetion_A;            /**< 定时器A重复计数值 */
    uint16_t compare1;              /**< 比较通道1值 */
    uint16_t compare2;              /**< 比较通道2值 */
    uint16_t compare3;              /**< 比较通道3值 */
    uint16_t period_B;              /**< 定时器B周期值 */
    uint16_t repetion_B;            /**< 定时器B重复计数值 */
} __attribute__((aligned(4))) burst_dma_t;

/**
 * @brief 脉冲输出模式枚举
 */
typedef enum {
    PULSE_MODE_NORMAL = 0,          /**< 正常模式 */
    PULSE_MODE_ECG_SYNC = 1         /**< 心电同步触发模式 */
} pulse_mode_t;

/**
 * @brief 脉冲输出状态枚举
 */
typedef enum {
    PULSE_STATUS_IDLE = 0,          /**< 空闲状态 */
    PULSE_STATUS_RUNNING = 1,       /**< 运行状态 */
    PULSE_STATUS_ERROR = 2          /**< 错误状态 */
} pulse_status_t;

/**
 * @brief 脉冲输出状态上报结构体
 */
typedef struct {
    pulse_mode_t   mode;            /**< 输出模式 */
    pulse_status_t status;          /**< 输出状态 */
    uint16_t       count;           /**< 输出次数 */
} pulse_report_t;

/*------------------------------ variable declarations -----------------------*/

/*------------------------------ function declarations -----------------------*/

void pulse_engine_init(void);
int32_t pulse_engine_set_mode(pulse_mode_t mode);
int32_t pulse_engine_get_mode(pulse_mode_t* mode);
int32_t pulse_engine_set_seq_param(const pulse_params_t* p);
int32_t pulse_engine_get_seq_param(pulse_params_t* p, uint16_t* total_groups);
int32_t pulse_engine_set_sync_param(const ecg_sync_cfg_t* config);
int32_t pulse_engine_get_sync_param(ecg_sync_cfg_t* config);
int32_t pulse_engine_start(void);
int32_t pulse_engine_stop(void);
int32_t pulse_engine_get_status(pulse_report_t* report);
void pulse_engine_notify_output_complete(void);

#ifdef __cplusplus
}
#endif

#endif /* PULSE_ENGINE_H */

/******************************* End Of File **********************************/

