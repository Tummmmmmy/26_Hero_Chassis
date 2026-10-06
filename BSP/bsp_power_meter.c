/**
 * @file bsp_power_meter.c
 * @brief 功率计板级驱动
 */

#include "bsp_power_meter.h"
#include "cmsis_os2.h"
#include <string.h>

power_meter_data_t power_meter;



//功率计初始化
void power_meter_init(void)
{
    power_meter.voltage = 0.0f;
    power_meter.current = 0.0f;
    power_meter.power = 0.0f;
    power_meter.last_update_time = 0;
    power_meter.offline_flag = 1;//默认离线
}




/**
  * @brief 解析反馈的功率计数据
  */
void power_meter_decode(uint8_t *rx_data)
{
    typedef union
    {
        struct
        {
            float voltage_mV;
            float current_mA;
        } pack;
        uint8_t data_buffer[8];
    } power_meter_union_t;

    power_meter_union_t temp_union;

    memcpy(temp_union.data_buffer, rx_data, 8);

    //单位换算
    power_meter.voltage = temp_union.pack.voltage_mV / 1000.0f;//mV -> V
    power_meter.current = temp_union.pack.current_mA / 1000.0f;//mA -> A
    //计算功率
    power_meter.power = power_meter.voltage * power_meter.current;

    power_meter.offline_flag = 0;
    power_meter.last_update_time = osKernelGetTickCount();
}



/**
  * @brief  功率计离线检测
  * @param  timeout_ms: 超时时间(ms)
  */
void power_meter_check_offline(uint32_t timeout_ms)
{
    uint32_t current_tick = osKernelGetTickCount();
    if (current_tick - power_meter.last_update_time > timeout_ms)
    {
        power_meter.offline_flag = 1;
    }
}

