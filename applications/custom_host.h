/**
  ******************************************************************************
  * @file        : custom_host.h
  * @brief       : 自定义协议主机接口
  * @details     本文件定义了自定义协议主机的接口函数。
  *              主机负责向从机发送命令、接收响应并处理。
  * @attention   使用前需要先调用 host_proto_init() 进行初始化
  ******************************************************************************
  */

#ifndef CUSTOM_HOST_H
#define CUSTOM_HOST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/**
 * @defgroup 主机设备地址定义
 * @{
 */
#define HOST_DEVICE_ADDR    (0x01U)  /**< 主机设备地址 */
/**
 * @}
 */

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化主机协议解析器
 * @retval 0 成功
 * @retval <0 失败（错误码）
 * @details 初始化串口设备、协议解析器和相关回调函数
 */
int host_proto_init(void);

/**
 * @brief 轮询主机协议解析器
 * @details 从串口FIFO中读取数据并送入协议解析器处理。
 *          建议在主循环或任务中定期调用此函数。
 */
void host_proto_poll(void);

/**
 * @brief 发送命令帧（带响应等待）
 * @param dev 目标设备地址
 * @param cmd 命令码
 * @param mod 模块标识
 * @param data 数据载荷指针（可为NULL）
 * @param data_len 数据载荷长度
 * @retval >=0 成功接收到响应的数据长度
 * @retval <0 失败（错误码，如-ETIMEDOUT表示超时）
 * @details 发送命令并等待响应，支持超时重试
 */
int host_send_cmd(uint8_t dev, uint8_t cmd, uint8_t mod,
                  const uint8_t *data, uint16_t data_len);

/**
 * @brief 获取最后一次接收的响应数据
 * @param buf 输出缓冲区
 * @param buf_size 缓冲区大小
 * @retval >=0 响应数据长度
 * @retval -1 无响应数据或缓冲区太小
 */
int host_get_last_response(uint8_t *buf, uint16_t buf_size);

/**
 * @brief 主动发送握手指令给指定从机
 * @param dev 目标从机设备地址
 * @retval 0 成功
 * @retval <0 失败（错误码）
 */
int host_send_handshake(uint8_t dev);

/**
 * @brief 获取从机在线状态
 * @param dev 从机设备地址
 * @retval 1 从机在线
 * @retval 0 从机离线或未知
 */
int host_is_slave_online(uint8_t dev);

/**
 * @brief 获取已连接的从机数量
 * @return 从机数量
 */
uint8_t host_get_slave_count(void);

/**
 * @brief 获取从机列表
 * @param buf 输出缓冲区（存放设备地址）
 * @param buf_size 缓冲区大小
 * @retval >=0 实际从机数量
 * @retval -1 参数错误
 */
int host_get_slave_list(uint8_t *buf, uint8_t buf_size);

/**
 * @brief 设置握手请求回调函数
 * @param cb 回调函数指针
 */
void host_set_handshake_callback(void (*cb)(uint8_t dev));

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_HOST_H */