/**
 * @file bsp_SuperCap.c
 * @brief 超级电容板级驱动
 */

#include "bsp_SuperCap.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include <string.h>
#include "bsp_power_meter.h"
#include "RM_JudgeSystem.h"
#include "gimbal_link.h"


supercap_controller_t g_supercap;


//CAN指令发送
static void supercap_send_cmd(FDCAN_HandleTypeDef* hfdcan)
{
    //0x601
    BSP_CAN_Send(hfdcan, SUPERCAP_TX_ID, g_supercap.tx_data.data, FDCAN_DLC_BYTES_8);
}



//初始化超电
void supercap_init(void)
{
    memset(&g_supercap, 0, sizeof(supercap_controller_t));
    g_supercap.offline_flag = true; //默认离线
}



//CAN报文解析
void supercap_rx_decode(uint8_t* rx_buf, uint8_t dlc)
{
    if (dlc > 8) dlc = 8;
    memcpy(g_supercap.rx_data.data, rx_buf, dlc);

    g_supercap.last_update_time = osKernelGetTickCount();
    g_supercap.offline_flag = false;
}




void SuperCap_Task(void const * argument)
{
    //等待系统稳定
    osDelay(2000);
	
    for(;;)
    {
        //离线检测
        if(osKernelGetTickCount() - g_supercap.last_update_time > SUPERCAP_OFFLINE_TIMEOUT_MS)
        {
            g_supercap.offline_flag = true;
        }

        //离线检测
        if(chassis.mode == 0 || g_supercap.offline_flag)
        {
            g_supercap.tx_data.pack.charge_enable = SUPERCAP_CHARGE_DISABLE;
            g_supercap.tx_data.pack.is_cap_output = SUPERCAP_OUTPUT_DISABLE;
            g_supercap.tx_data.pack.charge_power = 0.0f;
        }
        else//正常工作
        {
            //放电决策
            if(chassis.supercap_req == 1 || chassis.climb_mode ==2 || chassis.climb_mode ==1)//|| (chassis.key_shift && g_supercap.rx_data.pack.cap_level > 25)) 
            {
                //是否满足放电条件
                if(g_supercap.rx_data.pack.cap_usable && g_supercap.rx_data.pack.cap_level > SUPERCAP_MIN_OUTPUT_LEVEL)
								{
                    g_supercap.tx_data.pack.is_cap_output = SUPERCAP_OUTPUT_ENABLE;//开启超电
                } 
								else
							  {
                    
                    g_supercap.tx_data.pack.is_cap_output = SUPERCAP_OUTPUT_DISABLE;//关闭超电
                }
            }
            else
            {
                g_supercap.tx_data.pack.is_cap_output = SUPERCAP_OUTPUT_DISABLE;
            }

						
            //充电决策
            float charge_power_target = 0.0f;
            float referee_limit = ext_game_robot_state.data.chassis_power_limit;

            if(g_supercap.tx_data.pack.is_cap_output == SUPERCAP_OUTPUT_ENABLE)//正在放电
            {
							  
                charge_power_target = referee_limit * 0.9f;//0.95
            }
            else//没有放电
            {
								float current_buffer = ext_power_heat_data.data.chassis_power_buffer;

                const float CHARGE_KP = 30.0f;//每高出1J，增加30W充电功率
                float raw_charge_target = 0.0f;

                if(current_buffer > 55.99f)
                {
                    raw_charge_target = (current_buffer - 56.0f) * CHARGE_KP;//缓冲能量越高，充得越快
                }
                else
                {
                    raw_charge_target = 0.1f; 
                }
                
								
								//充电功率限幅
                float safe_max_charge = referee_limit;
								
								if(safe_max_charge > 120.0f) safe_max_charge = 120.0f;
                if(safe_max_charge < 0) safe_max_charge = 0;

                if(raw_charge_target > safe_max_charge)
								{
                    raw_charge_target = safe_max_charge;
                }

                static float filtered_charge_target = 0.0f;

                filtered_charge_target = (filtered_charge_target * 0.9f) + (raw_charge_target * 0.1f);
                
                charge_power_target = filtered_charge_target;
							}
						
							if(charge_power_target < 0.0f)   charge_power_target = 0.0f;
							if(charge_power_target > 120.0f) charge_power_target = 120.0f;
							g_supercap.tx_data.pack.charge_power = charge_power_target;
              g_supercap.tx_data.pack.charge_enable = SUPERCAP_CHARGE_ENABLE;
        }
        //发送指令
        supercap_send_cmd(&hfdcan2);

        osDelay(50);
    }
}

