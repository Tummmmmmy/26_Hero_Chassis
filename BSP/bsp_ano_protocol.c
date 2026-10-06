/**
 * @file bsp_ano_protocol.c
 * @brief 匿名上位机 & VOFA+ 通信协议模块实现
 *
 */

#include "bsp_ano_protocol.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os2.h"


// ======================== 模块私有定义与变量 ========================


#define COMM_UART_HANDLE (&huart10)


//定义一个静态的发送缓冲区
#define TX_BUFFER_SIZE 512

__align(32) static uint8_t tx_buffer[TX_BUFFER_SIZE];

extern osMessageQueueId_t anoPrintQueueHandle;


// ======================== 底层协议打包与发送函数 ========================


//根据消息类型，打包并启动DMA发送
static void Pack_And_Send_DMA(const Print_Msg_t* msg)
{
    uint16_t frame_len = 0;

    switch (msg->data_type)
    {
        case DATA_TYPE_ANO_WAVE://匿名上位机
        {
            //1.计算数据长度
            uint8_t data_len = msg->content.wave_pack.num_floats * 4;
            frame_len = data_len + 8;
            if(frame_len > TX_BUFFER_SIZE) return;

            //2.填充协议帧头
            tx_buffer[0] = 0xAB;
            tx_buffer[1] = 0x01;
            tx_buffer[2] = 0xFE;
            tx_buffer[3] = msg->content.wave_pack.frame_id;
            tx_buffer[4] = data_len;
            tx_buffer[5] = 0;

            //3.拷贝数据内容
            memcpy(&tx_buffer[6], msg->content.wave_pack.float_data, data_len);

            //4.计算并填充校验和,保证数据在传输过程中没有出错
            uint8_t sum_check = 0;
            uint8_t add_check = 0;
            for(int i = 0; i < frame_len - 2; ++i)
            {
                sum_check += tx_buffer[i];
                add_check += sum_check;
            }
            tx_buffer[frame_len - 2] = sum_check;
            tx_buffer[frame_len - 1] = add_check;
            break;
        }
        case DATA_TYPE_VOFA_TEXT://VOFA+
        {
            frame_len = strlen(msg->content.text_buffer);
            if (frame_len > TX_BUFFER_SIZE) return;
            memcpy(tx_buffer, msg->content.text_buffer, frame_len);
            break;
        }
    }

    if(frame_len > 0)
    {

        SCB_CleanDCache_by_Addr((uint32_t*)tx_buffer, TX_BUFFER_SIZE);
        //启动DMA发送
        HAL_UART_Transmit_DMA(COMM_UART_HANDLE, tx_buffer, frame_len);
    }
    else
    {
        osSemaphoreRelease(anoTxSemHandle); //如果无数据，直接释放信号量
    }
}


// ======================== 供外部调用的“生产者”函数 ========================


//打印匿名助手的波形
void ANO_Print_Wave(uint8_t frame_id, uint8_t num_floats, ...)
{
    if(num_floats == 0 || num_floats > MAX_WAVE_FLOATS) return;

    Print_Msg_t msg_to_queue;//创建一个临时的消息结构体
    msg_to_queue.data_type = DATA_TYPE_ANO_WAVE;
    msg_to_queue.content.wave_pack.frame_id = frame_id;
    msg_to_queue.content.wave_pack.num_floats = num_floats;

    va_list args;
    va_start(args, num_floats);
    for (int i = 0; i < num_floats; ++i)
    {
        msg_to_queue.content.wave_pack.float_data[i] = (float)va_arg(args, double);
    }
    va_end(args);

    //将打包好的消息放入队列,超时时间为0(不等待),如果队列满了，消息会被丢弃
             //(放进哪个队列的句柄,待发送数据地址,0,超时时间)
    osMessageQueuePut(anoPrintQueueHandle, &msg_to_queue, 0, 0);
}



//打印VOFA+波形
void VOFA_Print_Wave(uint8_t num_floats, ...)
{
    if(num_floats == 0 || num_floats > 20) return;

    Print_Msg_t msg_to_queue;
    msg_to_queue.data_type = DATA_TYPE_VOFA_TEXT;
    char* buffer = msg_to_queue.content.text_buffer;
    int offset = 0;

    va_list args;
    va_start(args, num_floats);
    for (int i = 0; i < num_floats; ++i)
    {
        float val = (float)va_arg(args, double);
        offset += snprintf(buffer + offset, MAX_TEXT_BUFFER_SIZE - offset, "%f,", val);
    }
    va_end(args);

    if (offset > 0)
    {
        buffer[offset - 1] = '\n';
    }

    osMessageQueuePut(anoPrintQueueHandle, &msg_to_queue, 0, 0);
}


// ======================== RTOS 任务与回调函数实现 ========================


//统一的打印任务
void Print_Task(void const *argument)
{
    //1.在任务开始时，先获取一次信号量，使计数值变为0，确保第一次发送前，信号量是不可用的
    osSemaphoreAcquire(anoTxSemHandle, 0);

    Print_Msg_t msg_to_print;

    for(;;)
    {
        //2.阻塞式地等待获取打印队列中的消息
        if(osMessageQueueGet(anoPrintQueueHandle, &msg_to_print, NULL, osWaitForever) == osOK)
        {
            //3.打包并启动DMA发送
            Pack_And_Send_DMA(&msg_to_print);

            //4.等待DMA传输完成
            //任务将在此处阻塞，直到中断释放信号量(获取哪个信号量,超时时间)
            osSemaphoreAcquire(anoTxSemHandle, osWaitForever);
        }
    }
}


//UART发送完成中断回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    //判断是否是匿名通信的串口
    if(huart->Instance == COMM_UART_HANDLE->Instance)
    {
        //释放信号量，唤醒正在等待的Print_Task
        osSemaphoreRelease(anoTxSemHandle);
    }
}
