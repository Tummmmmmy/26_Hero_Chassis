/**
 * @file bsp_can.c
 * @brief FDCAN 硬件抽象层
 *
 */

#include "bsp_can.h"
#include <string.h>

//FDCAN通信初始化
void BSP_FDCAN_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;//接收所有标准帧
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;//掩码模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x0000; //ID
    sFilterConfig.FilterID2 = 0x0000; // Mask=0, 接收所有

    //FDCAN1
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    //FDCAN2
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    //FDCAN3
    HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}


//CAN接收中断回调
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    //判断是否有新消息
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            //只处理标准帧
            if(rx_header.IdType == FDCAN_STANDARD_ID)
            {
                CAN_Rx_Message_t can_msg;
                can_msg.hfdcan = hfdcan;
                can_msg.can_id = rx_header.Identifier;
                memcpy(can_msg.data, rx_data, 8);

                osMessageQueuePut(canRxQueueHandle, &can_msg, 0, 0);
            }
        }
    }
}


//底层发送一帧
void BSP_CAN_Send(FDCAN_HandleTypeDef* hfdcan, uint32_t std_id, uint8_t* data, uint32_t dlc)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = std_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = dlc;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
    }
}
