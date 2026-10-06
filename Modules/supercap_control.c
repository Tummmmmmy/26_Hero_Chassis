/**
 * @file supercap_control.c
 * @brief 超级电容充放电决策
 *
 * 逻辑与原 SuperCap_Task 内决策部分一致:
 * 底盘失能/超电离线时充放全禁; 满足放电条件时开超电输出;
 * 未放电时按缓冲能量计算充电功率。
 */

#include "supercap_control.h"
#include "bsp_SuperCap.h"
#include "gimbal_link.h"
#include "RM_JudgeSystem.h"

//超电充放电决策
void SuperCap_Control_Update(void)
{
    supercap_cmd_t cmd = {0};

    //底盘未使能 或 超电离线: 充/放全禁
    if(chassis.mode == 0 || supercap_is_offline())
    {
        cmd.charge_enable = SUPERCAP_CHARGE_DISABLE;
        cmd.is_cap_output = SUPERCAP_OUTPUT_DISABLE;
        cmd.charge_power  = 0.0f;
    }
    else//正常工作
    {
        //放电决策
        if(chassis.supercap_req == 1 || chassis.climb_mode == 2 || chassis.climb_mode == 1)//|| (chassis.key_shift && supercap_get_cap_level() > 25))
        {
            //是否满足放电条件
            if(supercap_get_cap_usable() && supercap_get_cap_level() > SUPERCAP_MIN_OUTPUT_LEVEL)
            {
                cmd.is_cap_output = SUPERCAP_OUTPUT_ENABLE;//开启超电
            }
            else
            {
                cmd.is_cap_output = SUPERCAP_OUTPUT_DISABLE;//关闭超电
            }
        }
        else
        {
            cmd.is_cap_output = SUPERCAP_OUTPUT_DISABLE;
        }


        //充电决策
        float charge_power_target = 0.0f;
        float referee_limit = ext_game_robot_state.data.chassis_power_limit;

        if(cmd.is_cap_output == SUPERCAP_OUTPUT_ENABLE)//正在放电
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
        cmd.charge_power  = charge_power_target;
        cmd.charge_enable = SUPERCAP_CHARGE_ENABLE;
    }

    supercap_set_command(&cmd);
}
