# 脉冲发生方案设计说明

## 1. 概述

本文档详细说明脉冲发生系统的硬件和软件设计方案。系统基于STM32G4系列微控制器的高分辨率定时器（HRTIM），实现高精度、可配置的脉冲序列输出功能。

### 1.1 系统特点

- **高精度**：使用HRTIM实现纳秒级精度的脉冲宽度控制
- **灵活配置**：支持单极性和双极性脉冲，可配置多群、多串、多周期脉冲序列
- **安全保护**：集成硬件故障保护机制，支持过流保护
- **心电同步**：支持心电R波触发模式，实现与心电信号的同步输出
- **实时性**：硬件自动控制，最小化CPU干预

### 1.2 脉冲序列结构

脉冲序列采用四级层次结构：

```
脉冲序列 (Sequence)
  └── 群 (Group) - 最多20个群，群间有间隙
      └── 串 (Train) - 每群最多300串，串间有间隙
          └── 周期 (Period) - 每串最多300个周期
              └── 脉冲 (Pulse) - 单周期内的脉冲波形
                  ├── 正脉冲 (Positive Pulse)
                  └── 负脉冲 (Negative Pulse，可选)
```

## 2. 硬件设计

### 2.1 核心硬件资源

#### 2.1.1 HRTIM1（高分辨率定时器）

**功能**：生成高精度脉冲波形

**主要特性**：
- 时钟频率：170 MHz（系统时钟）
- 分辨率：0.68 ns（HRTIM_RESOLUTION = 0.68f）
- 支持突发模式（Burst Mode）
- 支持故障保护（Fault Protection）
- 支持DMA传输

**配置要点**：

1. **DLL校准**
   - 模式：连续校准（LL_HRTIM_DLLCALIBRATION_MODE_CONTINUOUS）
   - 速率：Rate 3
   - 目的：提高定时精度，减少时钟抖动

2. **Timer A配置**
   - 工作模式：连续计数模式（LL_HRTIM_MODE_CONTINUOUS）
   - 计数方向：向上计数（LL_HRTIM_COUNTING_MODE_UP）
   - 预分频：×4（LL_HRTIM_PRESCALERRATIO_MUL4）
   - 周期：0xFFFB（初始值，运行时动态配置）
   - 重复计数：0（初始值，运行时动态配置）
   - 更新门控：DMA突发模式（LL_HRTIM_UPDATEGATING_DMABURST）
   - 突发模式选项：复位计数器（LL_HRTIM_BURSTMODE_RESETCOUNTER）

3. **Timer B配置**
   - 工作模式：连续计数模式
   - 预分频：×4
   - 周期：与Timer A相同（用于同步）
   - 重复计数：0（方案2中不再使用硬件计数）
   - 更新门控：DMA突发模式

4. **输出通道配置**

   **TA1输出（PA8）**：
   - 极性：正极性（LL_HRTIM_OUT_POSITIVE_POLARITY）
   - 置位源：Timer A比较1（LL_HRTIM_OUTPUTSET_TIMCMP1）
   - 复位源：Timer A比较2 或 周期结束（LL_HRTIM_OUTPUTRESET_TIMCMP2|LL_HRTIM_OUTPUTRESET_TIMPER）
   - 空闲模式：突发模式时空闲（LL_HRTIM_OUT_IDLE_WHEN_BURST）
   - 空闲电平：无效（LL_HRTIM_OUT_IDLELEVEL_INACTIVE）
   - 故障状态：无效（LL_HRTIM_OUT_FAULTSTATE_INACTIVE）

   **TA2输出（PA9）**：
   - 极性：正极性
   - 置位源：Timer A比较3（LL_HRTIM_OUTPUTSET_TIMCMP3）
   - 复位源：周期结束（LL_HRTIM_OUTPUTRESET_TIMPER）
   - 空闲模式：突发模式时空闲
   - 空闲电平：无效
   - 故障状态：无效

5. **突发模式（Burst Mode）配置**
   - 模式：单次触发（LL_HRTIM_BM_MODE_SINGLESHOT）
   - 时钟源：HRTIM频率（LL_HRTIM_BM_CLKSRC_FHRTIM）
   - 触发源：Timer A重复中断（LL_HRTIM_BM_TRIG_TIMA_REPETITION）
   - 预分频：÷512（LL_HRTIM_BM_PRESCALER_DIV512）
   - 频率：332031 Hz（HRTIM_BUSTMODE_FREQ_HZ）
   - 用途：控制串间隙时间

#### 2.1.2 GPIO配置

**引脚分配**：
- **PA8**：HRTIM1_CHA1（TA1输出）
  - 模式：复用功能（LL_GPIO_MODE_ALTERNATE）
  - 复用功能：AF13（HRTIM1）
  - 速度：非常高（LL_GPIO_SPEED_FREQ_VERY_HIGH）
  - 输出类型：推挽输出（LL_GPIO_OUTPUT_PUSHPULL）
  - 上拉/下拉：无（LL_GPIO_PULL_NO）

- **PA9**：HRTIM1_CHA2（TA2输出）
  - 配置与PA8相同

#### 2.1.3 故障保护（Fault Protection）

**目的**：在检测到过流等故障时，立即停止脉冲输出，保护硬件和负载。

**配置**：

1. **Fault 4（对应内部比较器COMP1）**
   - 故障源：内部（LL_HRTIM_FLT_SRC_INTERNAL）
   - 极性：高电平有效（LL_HRTIM_FLT_POLARITY_HIGH）
   - 滤波：无（LL_HRTIM_FLT_FILTER_NONE）
   - 使能状态：已使能

2. **Fault 5（对应内部比较器COMP3）**
   - 配置与Fault 4相同

3. **故障应用到Timer A**
   - 使能Fault 4和Fault 5对Timer A的控制
   - 故障发生时，输出立即变为无效状态

4. **中断配置**
   - 中断向量：HRTIM1_FLT_IRQHandler
   - 优先级：0（最高优先级）
   - 用途：记录故障事件，用于调试和状态监控

#### 2.1.4 辅助定时器

**TIM6（群间隙定时器）**：
- 用途：控制群与群之间的间隙时间
- 时钟频率：170 MHz（TIMER_CLOCK_FREQUENCY）
- 工作模式：单次触发
- 中断：周期中断，用于启动下一群脉冲输出

**TIM4（心电R波检测定时器）**：
- 用途：输入捕获模式，检测心电R波
- 仅在心电同步模式下使用

**TIM7（心电触发延时定时器）**：
- 用途：R波检测后延时触发脉冲输出
- 仅在心电同步模式下使用

**TIM15（心电停止时间定时器）**：
- 用途：控制停止时间窗口
- 仅在心电同步模式下使用

### 2.2 硬件工作流程

#### 2.2.1 单周期脉冲生成

**单极性脉冲**（neg_pw = 0）：
```
时间轴：  |<--np_gap-->|<--pos_pw-->|
输出TA1:  _____________|____________|________
输出TA2:  ___________________________________
```

**双极性脉冲**（neg_pw > 0）：
```
时间轴：  |<--np_gap-->|<--pos_pw-->|<--pn_gap-->|<--neg_pw-->|
输出TA1:  _____________|____________|____________|____________|________
输出TA2:  _____________________________________________________|________
```

**硬件实现**：
- **TA1输出**：
  - 在Timer A计数到Compare1时置位（np_gap结束）
  - 在Timer A计数到Compare2时复位（pos_pw结束，单极性）或周期结束（双极性）
  
- **TA2输出**（仅双极性）：
  - 在Timer A计数到Compare3时置位（pn_gap结束）
  - 在周期结束时复位（neg_pw结束）

#### 2.2.2 脉冲串生成

**硬件机制**：
- Timer A的重复计数（Repetition Counter）控制每串的周期数
- 当Timer A完成指定周期数后，触发重复中断（REP中断）
- 突发模式（Burst Mode）在REP中断触发后，延时串间隙时间，然后重新启动Timer A

**流程**：
1. Timer A开始计数，输出脉冲周期
2. 完成periods_per_train个周期后，触发REP中断
3. 软件在中断中计数串数（train_cnt++）
4. 突发模式自动延时串间隙时间（train_gap）
5. 延时结束后，突发模式重新启动Timer A
6. 重复步骤1-5，直到完成train_per_group个串

#### 2.2.3 脉冲群生成

**硬件机制**：
- 每群的所有串输出完成后，停止HRTIM输出
- TIM6定时器控制群间隙时间（group_gap）
- TIM6超时后，重新启动HRTIM输出下一群

**流程**：
1. 配置当前群的脉冲参数到HRTIM寄存器
2. 启动HRTIM，输出当前群的所有串
3. 当前群所有串完成后，停止HRTIM
4. 启动TIM6，开始群间隙计时
5. TIM6超时后，配置下一群参数，重复步骤2-5
6. 所有群输出完成后，进入空闲状态

## 3. 软件设计

### 3.1 软件架构

#### 3.1.1 模块划分

```
┌─────────────────────────────────────────┐
│         脉冲引擎模块 (pulse_engine)      │
│  - 参数管理                              │
│  - 状态控制                              │
│  - 硬件配置                              │
│  - 输出控制                              │
└──────────────┬──────────────────────────┘
               │
    ┌──────────┴──────────┐
    │                     │
┌───▼────────┐    ┌───────▼────────┐
│ HRTIM BSP  │    │ 心电同步模块   │
│ (bsp_hrtim)│    │  (ecg_sync)    │
└────────────┘    └────────────────┘
```

#### 3.1.2 数据结构

**脉冲参数结构（pulse_params_t）**：
```c
typedef struct pulse_sequence {
    uint8_t  num_of_group;          // 群个数：1-20
    uint8_t  group_num;             // 群编号：1-20
    uint16_t group_gap;              // 群间隙：50-10000 ms
    uint16_t train_per_group;       // 每群串数：1-300
    uint16_t train_gap;             // 串间隙：100-10000 us（精度10us）
    uint16_t periods_per_train;     // 每串周期数：1-300
    uint16_t np_gap;                // 负-正脉冲间隙：ns
    uint16_t pos_pw;                // 正脉宽：ns
    uint16_t pn_gap;                // 正-负脉冲间隙：ns
    uint16_t neg_pw;                // 负脉宽：ns（0=单极性）
} pulse_params_t;
```

**硬件配置结构（burst_dma_t）**：
```c
typedef struct {
    uint16_t period_A;      // Timer A周期值
    uint16_t repetion_A;   // Timer A重复计数值
    uint16_t compare1;     // 比较通道1值
    uint16_t compare2;     // 比较通道2值
    uint16_t compare3;     // 比较通道3值
    uint16_t period_B;     // Timer B周期值
    uint16_t repetion_B;   // Timer B重复计数值（方案2中为0）
} burst_dma_t;
```

### 3.2 核心算法

#### 3.2.1 参数到硬件配置转换

**函数**：`pulse_out_set_hardware()`

**转换规则**：

1. **单极性脉冲**（neg_pw = 0）：
   ```c
   compare1 = np_gap * HRTIM_RESOLUTION
   compare2 = HRTIM_COMPARE_DISABLED_VALUE  // 禁用
   compare3 = HRTIM_COMPARE_DISABLED_VALUE  // 禁用
   period_A = (np_gap + pos_pw) * HRTIM_RESOLUTION
   ```

2. **双极性脉冲**（neg_pw > 0）：
   ```c
   compare1 = np_gap * HRTIM_RESOLUTION
   compare2 = (np_gap + pos_pw) * HRTIM_RESOLUTION
   compare3 = (np_gap + pos_pw + pn_gap) * HRTIM_RESOLUTION
   period_A = (np_gap + pos_pw + pn_gap + neg_pw) * HRTIM_RESOLUTION
   ```

3. **重复计数**：
   ```c
   repetion_A = periods_per_train - 1  // 硬件计数从0开始
   repetion_B = 0  // 方案2：不使用硬件计数
   ```

4. **串间隙**：
   ```c
   train_gap_buf[i] = tim_arr_from_us(train_gap)
   // 转换为突发模式的ARR值，精度10us
   ```

5. **群间隙**：
   ```c
   // 使用TIM6定时器，通过tim_pick_psc_arr_from_ns()计算PSC和ARR
   ```

#### 3.2.2 串计数方案（方案2）

**设计选择**：使用软件计数替代硬件计数

**原因**：
- Timer B的重复计数功能有限，无法满足复杂的串计数需求
- 软件计数更灵活，便于实现动态参数变化

**实现**：
- Timer A的REP中断触发时，软件计数器`train_cnt`递增
- 当`train_cnt >= train_per_group`时，表示当前群的所有串已完成
- 然后切换到下一群或结束输出

**中断处理流程**（HRTIM1_TIMA_IRQHandler）：
```
1. 检测REP中断标志
2. 清除中断标志
3. train_cnt++
4. 检查 train_cnt >= train_per_group？
   - 是：停止输出，group_cnt++，检查是否还有下一群
   - 否：继续等待下一串（突发模式自动处理串间隙）
```

#### 3.2.3 群切换流程

**函数**：`config_next_group_params()`

**流程**：
```
1. 配置Timer A的比较值和周期值
2. 配置Timer A的重复计数值
3. 配置Timer B的周期值（保持同步）
4. 配置突发模式的周期和比较值（串间隙）
5. 强制更新寄存器（ForceUpdate）
```

**群间隙处理**（TIM6中断）：
```
1. TIM6周期中断触发
2. 重新启动HRTIM输出（pulse_output_restart()）
3. 禁用TIM6（单次触发模式）
```

### 3.3 工作模式

#### 3.3.1 正常模式（PULSE_MODE_NORMAL）

**特点**：
- 立即启动脉冲输出
- 不依赖外部触发信号
- 适用于连续脉冲输出场景

**启动流程**：
```
1. pulse_engine_start() 被调用
2. 检查锁定状态、参数完整性、硬件配置状态
3. 设置状态为RUNNING
4. 初始化计数器（group_cnt=0, train_cnt=0）
5. 配置第一群参数
6. 启动HRTIM输出
```

#### 3.3.2 心电同步模式（PULSE_MODE_ECG_SYNC）

**特点**：
- 等待心电R波触发
- 支持触发延时、停止时间、R波间隔等参数
- 支持重复触发
- 适用于心脏起搏等医疗应用

**启动流程**：
```
1. pulse_engine_start() 被调用
2. 设置状态为RUNNING
3. 启动心电同步模块（ecg_sync_start()）
4. 等待R波检测（TIM4输入捕获）
5. R波检测到后，启动TIM7延时定时器
6. TIM7超时后，调用on_ecg_trigger_callback()
7. 启动脉冲输出（start_pulse_internal()）
```

**心电同步参数**（ecg_sync_cfg_t）：
```c
typedef struct {
    uint16_t trigger_delay_ms;  // 触发延时：0-300 ms
    uint16_t stop_delay_ms;     // 停止时间：200-2000 ms
    uint16_t interval_R;         // R波间隔：0-30
    uint16_t repeat_count;       // 重复次数：1-300
} ecg_sync_cfg_t;
```

**状态机**：
```
IDLE → WAIT_R_TRIGGER → WAIT_DELAY → PULSING → STOP_TIME → WAIT_INTERVAL → ...
```

### 3.4 中断处理

#### 3.4.1 HRTIM Timer A重复中断

**中断向量**：HRTIM1_TIMA_IRQHandler

**触发条件**：Timer A完成指定周期数（REP中断）

**处理流程**：
```
1. 检查REP中断标志
2. 清除中断标志
3. train_cnt++（软件计数串数）
4. 检查是否完成当前群的所有串
   - 是：
     a. train_cnt = 0
     b. 停止脉冲输出
     c. group_cnt++
     d. 检查是否还有下一群
       - 是：配置下一群参数，启动TIM6
       - 否：根据模式处理完成逻辑
   - 否：继续等待（突发模式自动处理串间隙）
```

#### 3.4.2 HRTIM故障中断

**中断向量**：HRTIM1_FLT_IRQHandler

**触发条件**：Fault 4或Fault 5被触发（过流保护）

**处理流程**：
```
1. 检查Fault 4标志
   - 清除标志
   - 记录故障事件（LOG）
2. 检查Fault 5标志
   - 清除标志
   - 记录故障事件（LOG）
```

**注意**：故障发生时，硬件自动停止输出，中断仅用于记录和监控。

#### 3.4.3 TIM6群间隙中断

**中断回调**：HAL_TIM_PeriodElapsedCallback（TIM6）

**触发条件**：TIM6计数到ARR值

**处理流程**：
```
1. 检查是否为TIM6中断
2. 重新启动HRTIM输出（pulse_output_restart()）
3. 禁用TIM6（单次触发模式）
```

### 3.5 安全机制

#### 3.5.1 锁定机制

**目的**：防止在硬件故障或未初始化时启动输出

**实现**：
- `g_control.locked`标志位
- 初始状态：锁定（1）
- 硬件初始化成功后：解锁（0）
- 过流保护触发后：锁定（1）

**检查点**：
- `pulse_engine_start()`：启动前检查锁定状态
- `start_pulse_internal()`：心电触发时检查锁定状态

#### 3.5.2 参数验证

**函数**：`param_is_valid()`

**验证项**：
- 群数量：1-20
- 群间隙：50-10000 ms
- 脉冲宽度：200-10000 ns
- 脉冲间隙：1000-30000 ns
- 每串周期数：1-300
- 串间隙：100-10000 us
- 每群串数：1-300

#### 3.5.3 状态检查

**启动前检查**：
- 锁定状态：必须未锁定
- 参数完整性：所有群参数必须已配置
- 硬件配置：硬件参数必须已配置
- 运行状态：必须处于IDLE状态

### 3.6 内存管理

#### 3.6.1 数据存储

**内部RAM**：
- `g_params`：脉冲参数管理结构（内部RAM）
- `g_control`：脉冲控制结构（内部RAM）

**外部SRAM**：
- `g_hw_config`：硬件配置缓冲区（外部SRAM，EXTSRAM宏定义）
  - `dma_buf[]`：每个群的硬件参数
  - `train_gap_buf[]`：每个群的串间隙参数

**原因**：
- 硬件配置数据量大（每个群需要多个参数）
- 外部SRAM容量更大，避免内部RAM溢出

#### 3.6.2 静态分配

**设计原则**：
- 不使用动态内存分配（malloc/free）
- 所有缓冲区静态分配
- 符合MISRA C:2012规范

## 4. 时序图

### 4.1 单极性脉冲时序

```
Timer A计数:  0 ──────────► Compare1 ──────────► Period ──► 0
输出TA1:      _____________|──────────|____________________|________
时间:         |<--np_gap-->|<--pos_pw-->|
```

### 4.2 双极性脉冲时序

```
Timer A计数:  0 ──► CMP1 ──► CMP2 ──► CMP3 ──► Period ──► 0
输出TA1:      ____|──────|____________________|____________________|________
输出TA2:      ________________________________|──────────|____________________|________
时间:         |np_gap|pos_pw|pn_gap|neg_pw|
```

### 4.3 脉冲串时序

```
Timer A:      [周期1][周期2]...[周期N] ──间隙── [周期1][周期2]...
输出:         ┌─────┐┌─────┐      ┌─────┐      ┌─────┐┌─────┐
              │脉冲 ││脉冲 │ ... │脉冲 │      │脉冲 ││脉冲 │
              └─────┘└─────┘      └─────┘      └─────┘└─────┘
时间:         |<--串1-->|<--train_gap-->|<--串2-->|
```

### 4.4 脉冲群时序

```
群1:          [串1][串2]...[串M] ──群间隙── [串1][串2]...[串M]
输出:         ┌───┐┌───┐    ┌───┐         ┌───┐┌───┐    ┌───┐
              │串 ││串 │ ...│串 │         │串 ││串 │ ...│串 │
              └───┘└───┘    └───┘         └───┘└───┘    └───┘
时间:         |<--群1-->|<--group_gap-->|<--群2-->|
```

## 5. 性能指标

### 5.1 精度指标

- **脉冲宽度精度**：0.68 ns（HRTIM分辨率）
- **脉冲宽度范围**：200 ns - 10000 ns
- **脉冲间隙精度**：0.68 ns
- **脉冲间隙范围**：1000 ns - 30000 ns
- **串间隙精度**：10 us（突发模式限制）
- **串间隙范围**：100 us - 10000 us
- **群间隙精度**：取决于TIM6配置（通常<1 ms）
- **群间隙范围**：50 ms - 10000 ms

### 5.2 容量指标

- **最大群数**：20
- **每群最大串数**：300
- **每串最大周期数**：300
- **总脉冲周期数**：最多 20 × 300 × 300 = 1,800,000 个周期

### 5.3 实时性指标

- **中断响应时间**：< 1 us（硬件自动处理）
- **参数切换时间**：< 10 us（寄存器更新）
- **故障保护响应时间**：< 100 ns（硬件自动）

## 6. 调试与测试

### 6.1 调试接口

- **日志输出**：通过LOG模块输出调试信息
- **状态查询**：`pulse_engine_get_status()`获取当前状态
- **参数查询**：`pulse_engine_get_seq_param()`获取配置参数

### 6.2 测试要点

1. **参数验证测试**：测试各种边界条件
2. **时序精度测试**：使用示波器测量实际输出时序
3. **故障保护测试**：模拟过流条件，验证保护机制
4. **心电同步测试**：使用模拟心电信号测试同步功能
5. **长时间运行测试**：验证系统稳定性

## 7. 注意事项

### 7.1 硬件注意事项

1. **DLL校准**：必须等待DLL校准完成后再使用HRTIM
2. **故障保护**：确保COMP1和COMP3正确配置并连接到HRTIM
3. **GPIO配置**：确保PA8和PA9配置为HRTIM复用功能
4. **时钟配置**：确保HRTIM时钟频率为170 MHz

### 7.2 软件注意事项

1. **参数顺序**：设置参数时必须按群编号顺序（1, 2, 3, ...）
2. **状态检查**：启动前必须检查锁定状态和参数完整性
3. **中断优先级**：HRTIM中断优先级应设置为最高（0）
4. **内存对齐**：硬件配置结构体需要4字节对齐（`__attribute__((aligned(4)))`）

### 7.3 安全注意事项

1. **过流保护**：必须正确配置过流检测电路
2. **锁定机制**：硬件故障后必须手动解锁才能重新启动
3. **参数范围**：必须严格验证所有参数在允许范围内
4. **状态机**：确保状态转换的正确性，避免非法状态

## 8. 版本历史

- **V1.0**：初始版本
  - 实现基本的脉冲输出功能
  - 支持单极性和双极性脉冲
  - 支持正常模式和心电同步模式
  - 实现故障保护机制
  - 采用软件串计数方案（方案2）

---

**文档版本**：V1.0  
**最后更新**：2025-01-XX  
**作者**：ZJY
