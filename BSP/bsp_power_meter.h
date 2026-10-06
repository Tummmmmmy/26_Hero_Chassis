/**
 * @file bsp_power_meter.h
 * @brief 功率计驱动接口
 *
 */

#ifndef BSP_POWER_METER_H
#define BSP_POWER_METER_H

#include <stdint.h>

//功率计反馈数据结构体
typedef struct
{
    float voltage;          //电压
    float current;          //电流
    float power;            //功率
    
    uint32_t last_update_time;//上次更新的时间戳(ms)
    uint8_t offline_flag;     //离线标志位,0:在线,1:离线
} power_meter_data_t;



extern power_meter_data_t power_meter;




void power_meter_init(void);

void power_meter_decode(uint8_t *rx_data);

void power_meter_check_offline(uint32_t timeout_ms);

#endif // BSP_POWER_METER_H
