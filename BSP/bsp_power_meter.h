/**
 * @file bsp_power_meter.h
 * @brief 功率计驱动接口 (FDCAN2, 报文 0x301)
 *
 * 功率计数据仅用于调试 / 功率模型数据采集, 不参与闭环控制;
 * 离线检测由 Monitor_Task 周期调用。
 */

#ifndef BSP_POWER_METER_H
#define BSP_POWER_METER_H

#include <stdint.h>
#include <stdbool.h>

//功率计初始化
void power_meter_init(void);

//解析功率计反馈报文(前4字节电压mV, 后4字节电流mA)
void power_meter_decode(const uint8_t* rx_data);

//功率计离线检测(超时置离线), 由 Monitor_Task 周期调用
void power_meter_check_offline(uint32_t timeout_ms);

//功率计是否离线
bool power_meter_is_offline(void);

//获取功率计功率(W)
float power_meter_get_power(void);

#endif // BSP_POWER_METER_H
