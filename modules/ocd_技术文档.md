# 过流检测模块技术文档

## 1. 概述

### 1.1 模块简介

过流检测模块（`ocd` - Over Current Detect）是一个用于实时监控电流并实现过流保护（OCP - Over Current Protection）的硬件抽象模块。该模块通过ADC DMA循环采样、硬件比较器触发和软件峰值检测相结合的方式，实现高精度、低延迟的过流检测功能。

### 1.2 主要特性

- **双通道监控**：支持两个独立的电流监控通道（CH1和CH2）
- **硬件触发**：使用STM32硬件比较器实现低延迟过流检测
- **DMA循环采样**：使用DMA实现高效的ADC数据采集，无需CPU干预
- **峰值检测**：在触发点附近搜索峰值，提供准确的过流值
- **事件回调**：支持过流事件回调，便于上层应用处理

### 1.3 技术指标

- **采样缓冲区大小**：每个通道256个采样点
- **峰值搜索窗口**：触发点前后各16个点（共32个点）
- **检测延迟**：硬件比较器触发，延迟极低（微秒级）
- **ADC分辨率**：12位（0-4095）

## 2. 系统架构

### 2.1 硬件连接

#### 2.1.1 通道1（CH1）配置
- **ADC**：ADC1
- **比较器**：COMP3
- **DAC**：DAC1（用于设置过流阈值）
- **输入引脚**：PA0（COMP3_INP）
- **DAC输出**：PA4（DAC1_OUT1）

#### 2.1.2 通道2（CH2）配置
- **ADC**：ADC2
- **比较器**：COMP1
- **DAC**：DAC3（用于设置过流阈值）
- **输入引脚**：PA1（COMP1_INP）
- **DAC输出**：内部连接（DAC3内部连接到COMP1）

### 2.2 工作原理

```
电流信号 → ADC采样 → DMA循环缓冲区
                ↓
        硬件比较器（实时监控）
                ↓
        过流触发 → 停止ADC → 标记触发位置
                ↓
        任务中处理 → 峰值搜索 → 回调通知
```

### 2.3 数据流

1. **采样阶段**：
   - ADC通过DMA循环采样，数据存储在256元素的缓冲区中
   - DMA自动循环覆盖旧数据，保持最新的采样数据

2. **监控阶段**：
   - 硬件比较器实时比较ADC输入与DAC阈值
   - 当输入超过阈值时，比较器产生上升沿中断

3. **触发处理**：
   - 中断服务程序立即停止对应通道的ADC DMA
   - 记录DMA停止时的位置作为触发点
   - 设置触发标志，等待任务处理

4. **峰值检测**：
   - 在任务中调用处理函数
   - 以触发点为中心，在前后各16个点的窗口内搜索峰值
   - 构建过流信息并调用回调函数

## 3. API接口

### 3.1 数据类型

#### 3.1.1 通道枚举
```c
typedef enum {
    OCD_CH1 = 0,  /**< ADC1通道（对应COMP3） */
    OCD_CH2 = 1   /**< ADC2通道（对应COMP1） */
} ocd_ch_t;
```

#### 3.1.2 过流信息结构体
```c
typedef struct {
    ocd_ch_t channel; /**< 检测通道 */
    uint16_t voltage_mv;          /**< 过流电压值（毫伏） */
} ocp_info_t;
```

#### 3.1.3 回调函数类型
```c
/** COMP+DAC检测方式的回调函数类型 */
typedef void (*ocp_info_cb_t)(const ocp_info_t *info);

/** IO引脚检测方式的回调函数类型 */
typedef void (*ocp_event_cb_t)(void);
```

### 3.2 初始化函数

#### 3.2.1 `ocd_init()`
```c
int ocd_init(void);
```

**功能描述**：初始化过流检测模块，配置内部缓冲区和通道信息。

**参数**：无

**返回值**：
- `0`：初始化成功
- 其他：初始化失败

**注意事项**：
- 必须在调用其他函数之前调用
- 使用内部定义的缓冲区，无需外部传入

**使用示例**：
```c
if (ocd_init() != 0) {
    LOG_E("Failed to initialize OCD");
    return;
}
```

### 3.3 控制函数

#### 3.3.1 `ocd_start()`
```c
int ocd_start(void);
```

**功能描述**：启动ADC DMA循环采样和比较器监控。

**参数**：无

**返回值**：
- `0`：启动成功
- `-ENODEV`：模块未初始化
- `-EIO`：启动失败

**功能说明**：
- 启动ADC1和ADC2的DMA循环采样
- 启动DAC1和DAC3，设置初始阈值（1985，约1.65V）
- 启动COMP1和COMP3比较器
- 使能比较器中断

**使用示例**：
```c
if (ocd_start() != 0) {
    LOG_E("Failed to start OCD");
    return;
}
```

#### 3.3.2 `ocd_stop()`
```c
void ocd_stop(void);
```

**功能描述**：停止ADC DMA循环采样。

**参数**：无

**返回值**：无

**注意事项**：
- 停止后不会自动清除触发标志
- 如需重新启动，直接调用`ocd_start()`

### 3.4 回调设置

#### 3.4.1 `ocd_set_ocp_callback()`
```c
void ocd_set_ocp_callback(ocp_info_cb_t callback);
```

**功能描述**：设置过流事件回调函数（COMP+DAC检测方式）。

**参数**：
- `callback`：回调函数指针，NULL表示清除回调

**返回值**：无

**回调函数说明**：
- 回调函数在任务上下文中调用，不在中断中
- 回调函数应尽快返回，避免阻塞
- 回调函数参数指向的`ocp_info_t`结构体在回调返回后可能失效
- 回调函数中会提供过流电压值（毫伏）

**使用示例**：
```c
static void on_ocp_info_callback(const ocp_info_t *info)
{
    if (info == NULL) {
        return;
    }

    LOG_I("OCP detected: CH%d, voltage=%d mV",
          info->channel, info->voltage_mv);

    /* 处理过流事件 */
    pulse_engine_stop();
}

/* 在初始化时设置回调 */
ocd_set_ocp_callback(on_ocp_info_callback);
```

#### 3.4.2 `ocd_set_ocp_event_callback()`
```c
void ocd_set_ocp_event_callback(ocp_event_cb_t callback);
```

**功能描述**：设置过流保护事件回调函数（IO引脚检测方式）。

**参数**：
- `callback`：回调函数指针，NULL表示清除回调

**返回值**：无

**回调函数说明**：
- 回调函数在GPIO中断上下文中调用，必须保持简短
- 回调函数应尽快返回，避免阻塞
- 此回调用于硬件IO引脚检测方式，当检测到过流时硬件会自动切断输出

**使用示例**：
```c
static void on_ocp_event_handler(void)
{
    LOG_I("Over-current detected! Stopping pulse engine...");

    /* 停止脉冲输出 */
    pulse_engine_stop();
}

/* 在初始化时设置回调 */
ocd_set_ocp_event_callback(on_ocp_event_handler);
```

### 3.5 事件处理函数

#### 3.5.1 `ocd_handle_ocp_trigger()`
```c
void ocd_handle_ocp_trigger(ocd_ch_t channel);
```

**功能描述**：处理过流触发事件（在比较器中断中调用）。

**参数**：
- `channel`：触发通道

**返回值**：无

**注意事项**：
- **此函数应在中断上下文中调用**
- 由`HAL_COMP_TriggerCallback()`自动调用，通常不需要手动调用
- 函数会立即停止对应通道的ADC DMA，保持缓冲区数据不变

#### 3.5.2 `ocd_process_ocp_event()`
```c
void ocd_process_ocp_event(void);
```

**功能描述**：处理过流事件（在任务中调用）。

**参数**：无

**返回值**：无

**功能说明**：
- 遍历所有通道，检查是否有触发标志
- 对触发的通道进行峰值搜索
- 将峰值ADC值转换为电压值（毫伏）
- 构建过流信息并调用回调函数
- 清除触发标志

**使用示例**：
```c
/* 在任务循环中定期调用 */
void monitor_task(void)
{
    while (1) {
        /* 处理过流事件 */
        ocd_process_ocp_event();

        /* 其他任务处理 */
        osDelay(10);
    }
}
```

### 3.6 IO引脚检测方式接口

#### 3.6.1 `ocd_get_ocp_pin_status()`
```c
uint8_t ocd_get_ocp_pin_status(void);
```

**功能描述**：获取过流保护引脚状态。

**参数**：无

**返回值**：
- `0`：安全（两个引脚都为低电平）
- `1`：不安全（至少一个引脚为高电平）

**注意事项**：
- 当两个引脚都为低电平时，表示硬件已恢复安全状态
- 用于确认硬件复位后是否真正安全

**使用示例**：
```c
uint8_t status = ocd_get_ocp_pin_status();
if (status == 0U) {
    LOG_D("OCP hardware is safe");
} else {
    LOG_W("OCP hardware is not safe");
}
```

#### 3.6.2 `ocd_reset_ocp_hardware()`
```c
int ocd_reset_ocp_hardware(void);
```

**功能描述**：硬件复位过流保护引脚。

**参数**：无

**返回值**：
- `0`：成功
- `-1`：失败

**注意事项**：
- 将复位引脚置高，用于硬件恢复
- 复位后需要检查引脚状态确认是否真正安全
- 建议复位后调用`ocd_get_ocp_pin_status()`确认安全状态

**使用示例**：
```c
/* 执行硬件复位 */
if (ocd_reset_ocp_hardware() == 0) {
    /* 检查是否真正安全 */
    if (ocd_get_ocp_pin_status() == 0U) {
        LOG_D("Hardware reset successful, system is safe");
    } else {
        LOG_W("Hardware reset failed, system is not safe");
    }
}
```

## 4. 实现细节

### 4.1 缓冲区管理

模块使用两个静态缓冲区：
- `adc1_dma_buf[256]`：通道1的DMA缓冲区
- `adc2_dma_buf[256]`：通道2的DMA缓冲区

缓冲区由DMA循环填充，当缓冲区满后自动从头开始覆盖。

### 4.2 DMA位置计算

当比较器触发时，需要确定DMA当前写入位置。通过读取DMA剩余传输计数，计算已传输数量：

```c
remaining = __HAL_DMA_GET_COUNTER(hdma);
transferred = buf_size - remaining;
pos = transferred % buf_size;
```

### 4.3 峰值搜索算法

峰值搜索在触发点前后各32个点的窗口内进行：

1. **计算搜索窗口**：
   - 起始位置：`center_pos - 32`（考虑循环）
   - 结束位置：`center_pos + 32`（考虑循环）

2. **处理边界情况**：
   - 如果窗口不跨越缓冲区边界，直接线性搜索
   - 如果窗口跨越边界，分两段搜索

3. **查找最大值**：
   - 遍历窗口内所有点，找到最大值及其位置

### 4.4 ADC值到电压值转换

ADC值通过以下公式转换为电压值（毫伏）：

```
voltage_mv = (adc_value * 3300) / 4095
```

其中：
- `adc_value`：ADC寄存器值（12位，0-4095）
- `3300`：ADC参考电压（3.3V = 3300mV）
- `4095`：ADC最大值（12位ADC）

转换后的电压值存储在`ocp_info_t.voltage_mv`中，单位是毫伏（mV）。

### 4.5 IO引脚检测方式

模块支持两种过流检测方式：

1. **COMP+DAC检测方式**：
   - 使用硬件比较器实时监控
   - 通过DAC设置可配置的阈值
   - 提供精确的过流电压值

2. **IO引脚检测方式**：
   - 使用硬件电路检测，固定阈值
   - 硬件自动切断输出
   - 通过GPIO中断通知软件
   - 需要软件复位引脚进行硬件恢复

两种方式在初始化时都会自动配置，可以同时使用。

### 4.6 中断处理流程

**COMP+DAC检测方式：**
```
COMP1_2_3_IRQHandler()
    ↓
HAL_COMP_IRQHandler()
    ↓
HAL_COMP_TriggerCallback()
    ↓
ocd_handle_ocp_trigger()
    ↓
停止ADC DMA + 记录触发位置 + 设置标志
```

**IO引脚检测方式：**
```
硬件过流 → OCP_POSITIVE_PIN_ID或OCP_NEGTIVE_PIN_ID置高
    ↓
GPIO上升沿中断
    ↓
ocp_io_handle()
    ↓
调用ocp_event_cb_t回调函数
```

## 5. 使用流程

### 5.1 基本使用流程

```c
/* 1. 初始化模块 */
if (ocd_init() != 0) {
    return -1;
}

/* 2. 设置回调函数 */
ocd_set_ocp_callback(on_ocp_info_callback);

/* 3. 启动监控 */
if (ocd_start() != 0) {
    return -1;
}

/* 4. 在任务中定期处理事件 */
void task_loop(void)
{
    while (1) {
        ocd_process_ocp_event();
        osDelay(10);
    }
}

/* 5. 停止监控（如需要） */
ocd_stop();
```

### 5.2 阈值设置

DAC阈值在`ocd_start()`中设置为4095（12位DAC，对应3.3V）。如需修改阈值，可在启动后调用：

```c
HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, threshold_value);
HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, threshold_value);
```

**阈值计算公式**：
```
阈值电压 = (threshold_value / 4095) × VREF
```

其中VREF为DAC参考电压（通常为3.3V）。

### 5.3 IO引脚检测方式使用

IO引脚检测方式在初始化时自动配置，使用方式如下：

```c
/* 1. 设置IO引脚检测方式的回调函数 */
ocd_set_ocp_event_callback(on_ocp_event_handler);

/* 2. 在回调函数中处理过流事件 */
static void on_ocp_event_handler(void)
{
    /* 停止脉冲输出 */
    pulse_engine_stop();

    /* 其他处理... */
}

/* 3. 确认安全后，复位硬件 */
if (ocd_reset_ocp_hardware() == 0) {
    /* 检查是否真正安全 */
    if (ocd_get_ocp_pin_status() == 0U) {
        /* 硬件已恢复安全状态 */
    }
}
```

## 6. 依赖关系

### 6.1 硬件依赖

- **ADC1**：用于通道1采样
- **ADC2**：用于通道2采样
- **DAC1**：用于通道1阈值设置
- **DAC3**：用于通道2阈值设置
- **COMP1**：用于通道2过流检测
- **COMP3**：用于通道1过流检测
- **DMA**：用于ADC数据传输
- **GPIO**：用于IO引脚检测方式（OCP_POSITIVE_PIN_ID、OCP_NEGTIVE_PIN_ID、OCP_RESET_PIN_ID）

### 6.2 软件依赖

- `bsp_adc.h`：ADC底层驱动
- `bsp_dac.h`：DAC底层驱动
- `bsp_comp.h`：比较器底层驱动
- `board.h`：板级配置（包含引脚定义）
- `gpio.h`：GPIO驱动
- `errno-base.h`：错误码定义
- `log.h`：日志系统

### 6.3 外部变量

模块依赖以下外部HAL句柄（需在BSP层初始化）：
- `hadc1`：ADC1句柄
- `hadc2`：ADC2句柄
- `hdac1`：DAC1句柄
- `hdac3`：DAC3句柄
- `hcomp1`：COMP1句柄
- `hcomp3`：COMP3句柄
- `hdma_adc1`：ADC1 DMA句柄
- `hdma_adc2`：ADC2 DMA句柄

## 7. 注意事项

### 7.1 线程安全

- `ocd_handle_ocp_trigger()`在中断中调用，必须保持简短
- `ocd_process_ocp_event()`应在任务中调用，不要在中断中调用
- 回调函数在任务上下文中执行，可以执行较复杂的操作

### 7.2 性能考虑

- DMA循环采样不占用CPU资源
- 峰值搜索算法时间复杂度为O(32)，计算量小
- 建议在10-50ms周期内调用`ocd_process_ocp_event()`

### 7.3 错误处理

- 初始化失败时应检查硬件配置
- 启动失败时应检查ADC/DAC/COMP是否已正确初始化
- 建议在回调函数中进行错误处理和状态上报

### 7.4 MISRA C合规性

- 代码遵循MISRA C:2012规范
- 使用C99标准
- 无动态内存分配
- 所有变量都有明确的类型和初始化

## 8. 故障排查

### 8.1 常见问题

**问题1：过流检测不触发**
- 检查DAC阈值设置是否正确
- 检查比较器是否已启动
- 检查中断是否已使能
- 检查ADC输入信号是否正常

**问题2：峰值检测不准确**
- 检查触发位置计算是否正确
- 检查缓冲区数据是否有效
- 检查搜索窗口大小是否合适

**问题3：回调函数不执行**
- 检查是否设置了回调函数
- 检查是否定期调用`ocd_process_ocp_event()`
- 检查触发标志是否被正确设置

### 8.2 调试建议

- 使用日志系统查看模块运行状态
- 在回调函数中添加断点，检查过流信息
- 检查DMA缓冲区数据，验证采样是否正常
- 使用示波器检查比较器输入信号

## 9. 版本历史

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| V1.0 | 20xx-xx-xx | 初始版本 |
| V1.1 | 20xx-xx-xx | 1. 整合IO引脚检测方式到OCD模块<br/>2. 修改ocp_info_t结构体：去除trigger_position和timestamp，peak_value改为voltage_mv<br/>3. 实现ADC值到电压值的转换<br/>4. 添加IO引脚检测方式的接口函数 |
| V1.2 | 20xx-xx-xx | 1. 模块重命名：current_monitor -> ocd<br/>2. 所有函数名和类型名更新为ocd_*前缀<br/>3. 添加阈值设置和获取接口函数 |

## 10. 参考资料

- STM32 HAL库用户手册
- STM32 ADC应用笔记
- STM32 DAC应用笔记
- STM32比较器应用笔记
- MISRA C:2012规范


