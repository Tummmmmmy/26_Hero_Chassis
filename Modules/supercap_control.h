/**
 * @file supercap_control.h
 * @brief 超级电容充放电决策接口
 *
 * 输入: 云台板指令(chassis)与裁判系统功率数据;
 * 输出: 超电充放电命令(通过 bsp_SuperCap 的 set_command 下发)。
 */

#ifndef SUPERCAP_CONTROL_H
#define SUPERCAP_CONTROL_H

//超电充放电决策(由 SuperCap_Task 每 50ms 调用)
void SuperCap_Control_Update(void);

#endif
