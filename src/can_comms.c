#include "can_comms.h"

HAL_StatusTypeDef CAN_Comms_Init(CAN_HandleTypeDef *hcan)
{
    hcan->Init.Prescaler            = 5;
    hcan->Init.Mode                 = CAN_MODE_NORMAL;
    hcan->Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan->Init.TimeSeg1             = CAN_BS1_15TQ;
    hcan->Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan->Init.TimeTriggeredMode    = DISABLE;
    hcan->Init.AutoBusOff           = ENABLE;
    hcan->Init.AutoWakeUp           = DISABLE;
    hcan->Init.AutoRetransmission   = ENABLE;
    hcan->Init.ReceiveFifoLocked    = DISABLE;
    hcan->Init.TransmitFifoPriority = DISABLE;

    HAL_StatusTypeDef st = HAL_CAN_Init(hcan);
    if (st != HAL_OK) return st;

    CAN_FilterTypeDef filter;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    st = HAL_CAN_ConfigFilter(hcan, &filter);
    if (st != HAL_OK) return st;

    st = HAL_CAN_Start(hcan);
    return st;
}

HAL_StatusTypeDef CAN_SendSensorData(CAN_HandleTypeDef *hcan,
                                     BME280_Data *data,
                                     uint8_t fan_state)
{
    CAN_TxHeaderTypeDef header;
    uint8_t payload[8];
    uint32_t mailbox;
    HAL_StatusTypeDef st;

    header.StdId              = CAN_ID_SENSOR_DATA;
    header.ExtId              = 0;
    header.IDE                = CAN_ID_STD;
    header.RTR                = CAN_RTR_DATA;
    header.DLC                = 8;
    header.TransmitGlobalTime = DISABLE;

    payload[0] = (uint8_t)(data->temperature_x100 & 0xFF);
    payload[1] = (uint8_t)((data->temperature_x100 >> 8) & 0xFF);
    payload[2] = (uint8_t)(data->humidity_x100 & 0xFF);
    payload[3] = (uint8_t)((data->humidity_x100 >> 8) & 0xFF);
    payload[4] = (uint8_t)(data->pressure_Pa & 0xFF);
    payload[5] = (uint8_t)((data->pressure_Pa >> 8) & 0xFF);
    payload[6] = (uint8_t)((data->pressure_Pa >> 16) & 0xFF);
    payload[7] = (uint8_t)((data->pressure_Pa >> 24) & 0xFF);

    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        if ((HAL_GetTick() - tick) > 10) return HAL_TIMEOUT;
    }

    st = HAL_CAN_AddTxMessage(hcan, &header, payload, &mailbox);
    if (st != HAL_OK) return st;

    header.StdId = CAN_ID_PRESSURE_DATA;
    header.DLC   = 2;

    payload[0] = fan_state;
    payload[1] = 0x00;

    tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        if ((HAL_GetTick() - tick) > 10) return HAL_TIMEOUT;
    }

    st = HAL_CAN_AddTxMessage(hcan, &header, payload, &mailbox);
    return st;
}

HAL_StatusTypeDef CAN_SendHeartbeat(CAN_HandleTypeDef *hcan, uint32_t uptime_sec)
{
    CAN_TxHeaderTypeDef header;
    uint8_t payload[4];
    uint32_t mailbox;

    header.StdId              = CAN_ID_HEARTBEAT;
    header.ExtId              = 0;
    header.IDE                = CAN_ID_STD;
    header.RTR                = CAN_RTR_DATA;
    header.DLC                = 4;
    header.TransmitGlobalTime = DISABLE;

    payload[0] = (uint8_t)(uptime_sec & 0xFF);
    payload[1] = (uint8_t)((uptime_sec >> 8) & 0xFF);
    payload[2] = (uint8_t)((uptime_sec >> 16) & 0xFF);
    payload[3] = (uint8_t)((uptime_sec >> 24) & 0xFF);

    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        if ((HAL_GetTick() - tick) > 10) return HAL_TIMEOUT;
    }

    return HAL_CAN_AddTxMessage(hcan, &header, payload, &mailbox);
}
