/**
 * @file bsp_ano_protocol.h
 * @brief 匿名上位机 / VOFA+ 打印协议接口
 *
 */

#ifndef BSP_ANO_PROTOCOL_H
#define BSP_ANO_PROTOCOL_H

#include "main.h"
#include <stdarg.h>

extern osSemaphoreId_t anoTxSemHandle;


/** @brief 打印数据类型 */
typedef enum {
    DATA_TYPE_ANO_WAVE,      //匿名助手
    DATA_TYPE_VOFA_TEXT      //VOFA+
} Data_Type_e;


#define MAX_WAVE_FLOATS      20   //定义一帧最多可以发送20个float，可根据需要调整
#define MAX_TEXT_BUFFER_SIZE 128  //文本数据最大长度128字节

/** @brief 打印消息 */
typedef struct
{
    //标志位，告诉底层如何打包和发送
    Data_Type_e data_type; 

    //联合体，所有成员共享同一块内存，极其节省RAM
    union
   {
        //匿名
        struct
        {
            uint8_t frame_id;     //功能码(0xF1-0xFA)
            uint8_t num_floats;   //本次发送的float数量
            float float_data[MAX_WAVE_FLOATS];//存储float数据的数组
        } wave_pack;

        //VOFA
        char text_buffer[MAX_TEXT_BUFFER_SIZE];

    } content;

} Print_Msg_t;


// ======================== 供外部调用的“生产者”函数 ========================

/**
  * @brief  【生产者】发送多个浮点数作为波形数据（线程安全）
  * @param  frame_id    功能码 (0xF1 - 0xFA)
  * @param  num_floats  浮点数的个数 (不能超过 MAX_ANO_FLOATS)
  * @param  ...         可变参数，传入要发送的float变量
  * @retval None
  */
void ANO_Print_Wave(uint8_t frame_id, uint8_t num_floats, ...);


/** @brief 发送 VOFA+ 兼容的文本格式波形数据 */
void VOFA_Print_Wave(uint8_t num_floats, ...);


// ======================== RTOS 任务函数声明 ========================

/** @brief 【消费者】统一的打印任务入口函数 */
void Ano_Print_Task(void const *argument);


#endif
