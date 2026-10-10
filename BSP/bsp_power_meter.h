/**
 * @file bsp_power_meter.h
 * @brief 功率计驱动接口 
 *
 */

#ifndef BSP_POWER_METER_H
#define BSP_POWER_METER_H

#include <stdint.h>
#include <stdbool.h>


//功率计初始化
void power_meter_init(void);

//解析功率计反馈报文
void power_meter_decode(const uint8_t* rx_data);

//功率计离线检测
void power_meter_check_offline(uint32_t timeout_ms);

//功率计是否离线
bool power_meter_is_offline(void);

//获取功率计功率(W)
float power_meter_get_power(void);

#endif // BSP_POWER_METER_H
