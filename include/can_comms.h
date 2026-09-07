#ifndef CAN_COMMS_H
#define CAN_COMMS_H

#include "stm32f4xx_hal.h"
#include "bme280.h"
#include <stdint.h>

#define CAN_ID_SENSOR_DATA      0x100
#define CAN_ID_PRESSURE_DATA    0x101
#define CAN_ID_HEARTBEAT        0x010

#define FAN_STATE_OFF            0x00
#define FAN_STATE_ON             0x01

HAL_StatusTypeDef CAN_Comms_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef CAN_SendSensorData(CAN_HandleTypeDef *hcan,
                                     BME280_Data *data,
                                     uint8_t fan_state);
HAL_StatusTypeDef CAN_SendHeartbeat(CAN_HandleTypeDef *hcan, uint32_t uptime_sec);

#endif
