//
// Created by 34285 on 2026/6/4.

#include "drv_can.h"

Struct_CAN_Manage_Object CAN1_Manage_Object = {0};
Struct_CAN_Manage_Object CAN2_Manage_Object = {0};

//CAN通信发送缓冲区
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN1_0x1FF_Tx_Data[8];
uint8_t CAN1_0x2FF_Tx_Data[8];
uint8_t CAN1_0x1FE_Tx_Data[8];
uint8_t CAN1_0x2FE_Tx_Data[8];

uint8_t CAN2_0x200_Tx_Data[8];
uint8_t CAN2_0x1FF_Tx_Data[8];
uint8_t CAN2_0x2FF_Tx_Data[8];
uint8_t CAN2_0x1FE_Tx_Data[8];
uint8_t CAN2_0x2FE_Tx_Data[8];

/**
 * @brief 分配CAN发送缓冲区
 *
 * @param hcan CAN编号
 * @param Motor_ID CAN ID
 * @return 缓冲区指针
 */
uint8_t *Allocate_Tx_Data(FDCAN_HandleTypeDef *hcan, Enum_RM_Motor_ID Motor_ID)
{
    uint8_t *tmp_tx_data_ptr = NULL;
    if (hcan->Instance == FDCAN1)
    {
        switch (Motor_ID)
        {
            case (RM_Motor_ID_0x201):
                {
                    tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[0]);
                }
                break;
            case (RM_Motor_ID_0x202):
                {
                    tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[2]);
                }
                break;
            case (RM_Motor_ID_0x203):
                {
                    tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[4]);
                }
                break;
            case (RM_Motor_ID_0x204):
                {
                    tmp_tx_data_ptr = &(CAN1_0x200_Tx_Data[6]);
                }
                break;
            case (RM_Motor_ID_0x205):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FF_Tx_Data[0]);
                }
                break;
            case (RM_Motor_ID_0x206):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FF_Tx_Data[2]);
                }
                break;
            case (RM_Motor_ID_0x207):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FF_Tx_Data[4]);
                }
                break;
            case (RM_Motor_ID_0x208):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FF_Tx_Data[6]);
                }
                break;
            case (RM_Motor_ID_0x209):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FE_Tx_Data[0]);
                }
                break;
            case (RM_Motor_ID_0x20A):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FE_Tx_Data[2]);
                }
                break;
            case (RM_Motor_ID_0x20B):
                {
                    tmp_tx_data_ptr = &(CAN1_0x1FE_Tx_Data[4]);
                }
                break;
            default:
                break;
        }
    }
    else if (hcan->Instance == FDCAN2)
    {
        switch (Motor_ID)
        {
            case (RM_Motor_ID_0x201):
                {
                    tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[0]);
                }
                break;
            case (RM_Motor_ID_0x202):
                {
                    tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[2]);
                }
                break;
            case (RM_Motor_ID_0x203):
                {
                    tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[4]);
                }
                break;
            case (RM_Motor_ID_0x204):
                {
                    tmp_tx_data_ptr = &(CAN2_0x200_Tx_Data[6]);
                }
                break;
            case (RM_Motor_ID_0x205):
                {
                    tmp_tx_data_ptr = &(CAN2_0x1FF_Tx_Data[0]);
                }
                break;
            case (RM_Motor_ID_0x206):
                {
                    tmp_tx_data_ptr = &(CAN2_0x1FF_Tx_Data[2]);
                }
                break;
            case (RM_Motor_ID_0x207):
                {
                    tmp_tx_data_ptr = &(CAN2_0x1FF_Tx_Data[4]);
                }
                break;
            case (RM_Motor_ID_0x208):
                {
                    tmp_tx_data_ptr = &(CAN2_0x1FF_Tx_Data[6]);
                }
                break;
            case (RM_Motor_ID_0x209):
                {
                    tmp_tx_data_ptr = &(CAN2_0x1FE_Tx_Data[0]);
                }
                break;
            case (RM_Motor_ID_0x20A):
                {
                    tmp_tx_data_ptr = &(CAN2_0x1FE_Tx_Data[2]);
                }
                break;
            case (RM_Motor_ID_0x20B):
                {
                    tmp_tx_data_ptr = &(CAN2_0x1FE_Tx_Data[4]);
                }
                break;
            default:
                break;
        }
    }
    return tmp_tx_data_ptr;
}

/**
 * @brief 初始化CAN总线
 *
 * @param hfdcan FDCAN句柄
 * @param Callback_Function 处理回调函数
 */
void CAN_Init(FDCAN_HandleTypeDef *hfdcan, CAN_Call_Back Callback_Function)
{
    HAL_FDCAN_Start(hfdcan);
    // 激活 FIFO0 和 FIFO1 的新消息接收中断
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

    if (hfdcan->Instance == FDCAN1)
    {
        CAN1_Manage_Object.CAN_Handler = hfdcan;
        CAN1_Manage_Object.Callback_Function = Callback_Function;
        RM_CAN_Filter_Mask_Config(&hfacan1, 0, FDCAN_FILTER_TO_RXFIFO0, 0x200, 0x7F0);
        RM_CAN_Filter_Mask_Config(&hfacan1, 1, FDCAN_FILTER_TO_RXFIFO1, 0x200, 0x7F0);
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        CAN2_Manage_Object.CAN_Handler = hfdcan;
        CAN2_Manage_Object.Callback_Function = Callback_Function;
        RM_CAN_Filter_Mask_Config(&hfacan2, 64, FDCAN_FILTER_TO_RXFIFO0, 0x200, 0x7F0);
        RM_CAN_Filter_Mask_Config(&hfacan2, 65, FDCAN_FILTER_TO_RXFIFO1, 0x200, 0x7F0);
    }
}

/**
 * @brief 配置CAN的滤波器
 *
 * @param hfdcan FDCAN句柄
 * @param filter_index 滤波器编号
 * @param filter_config 滤波器FIFO
 * @param Filter_ID ID
 * @param Mask_ID 屏蔽位
 */
void RM_CAN_Filter_Mask_Config(FDCAN_HandleTypeDef *hfdcan, uint32_t filter_index, uint32_t filter_config,
                               uint32_t Filter_ID, uint32_t Mask_ID)
{
    //自定义滤波器结构体名称
    FDCAN_FilterTypeDef FDCAN_filter_mask_structure;

    FDCAN_filter_mask_structure.IdType = FDCAN_STANDARD_ID;
    FDCAN_filter_mask_structure.FilterIndex = filter_index;
    FDCAN_filter_mask_structure.FilterType = FDCAN_FILTER_MASK;
    FDCAN_filter_mask_structure.FilterConfig = filter_config;
    FDCAN_filter_mask_structure.FilterID1 = Filter_ID;
    FDCAN_filter_mask_structure.FilterID2 = Mask_ID;
    FDCAN_filter_mask_structure.RxBufferIndex = 0;
    FDCAN_filter_mask_structure.IsCalibrationMsg = 0;

    HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN_filter_mask_structure);
}

/**
 * @brief 发送CAN数据帧
 *
 * @param hfdcan CAN编号
 * @param ID ID
 * @param Data 被发送的数据指针
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *Data)
{
    FDCAN_TxHeaderTypeDef tx_header;

    assert_param(hfdcan != NULL);

    tx_header.Identifier = ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // //假设你要发送 64 字节的 FD 报文，并且开启波特率切换提速
    // tx_header.DataLength = FDCAN_DLC_BYTES_64;     // 数据长度改为 64 字节
    // tx_header.FDFormat = FDCAN_FD_CAN;             // 开启 CAN-FD 格式
    // tx_header.BitRateSwitch = FDCAN_BRS_ON;        // 开启波特率切换 (数据段会按高速发送)

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, Data);
}

/**
 * @brief CAN的TIM定时器中断发送回调函数
 *
 */
void TIM_CAN_PeriodElapsedCallback(void)
{
    // CAN1总线电机
    CAN_Send_Data(&hfacan1, 0x200, CAN1_0x200_Tx_Data);
    // CAN_Send_Data(&hfacan1, 0x1FF, CAN1_0x1FF_Tx_Data);
    // CAN_Send_Data(&hfacan1, 0x2FF, CAN1_0x2FF_Tx_Data);
    // CAN_Send_Data(&hfacan1, 0x1FE, CAN1_0x1FE_Tx_Data);
    // CAN_Send_Data(&hfacan1, 0x2FE, CAN1_0x2FE_Tx_Data);

    // CAN2总线电机
    // CAN_Send_Data(&hfacan2, 0x200, CAN2_0x200_Tx_Data);
    // CAN_Send_Data(&hfacan2, 0x1FF, CAN2_0x1FF_Tx_Data);
    // CAN_Send_Data(&hfacan2, 0x2FF, CAN2_0x2FF_Tx_Data);
    // CAN_Send_Data(&hfacan2, 0x1FE, CAN2_0x1FE_Tx_Data);
    // CAN_Send_Data(&hfacan2, 0x2FE, CAN2_0x2FE_Tx_Data);
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_FDCAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_Manage_Object.Rx_Buffer.Header,
                               CAN1_Manage_Object.Rx_Buffer.Data);
        if (CAN1_Manage_Object.Callback_Function != NULL)
        {
            CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN2_Manage_Object.Rx_Buffer.Header,
                               CAN2_Manage_Object.Rx_Buffer.Data);
        if (CAN2_Manage_Object.Callback_Function != NULL)
        {
            CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
        }
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_FDCAN_RxFifo1MsgPendingCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN1_Manage_Object.Rx_Buffer.Header,
                             CAN1_Manage_Object.Rx_Buffer.Data);
        if (CAN1_Manage_Object.Callback_Function != NULL)
        {
            CAN1_Manage_Object.Callback_Function(&CAN1_Manage_Object.Rx_Buffer);
        }
    }
    else if (hfdcan->Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN2_Manage_Object.Rx_Buffer.Header,
                             CAN2_Manage_Object.Rx_Buffer.Data);
        if (CAN2_Manage_Object.Callback_Function != NULL)
        {
            CAN2_Manage_Object.Callback_Function(&CAN2_Manage_Object.Rx_Buffer);
        }
    }
}


