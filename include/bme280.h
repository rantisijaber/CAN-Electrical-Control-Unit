#ifndef BME280_H
#define BME280_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define BME280_I2C_ADDR         (0x76 << 1)

#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRESS_MSB    0xF7
#define BME280_REG_CALIB_00     0x88
#define BME280_REG_CALIB_26     0xE1

#define BME280_CHIP_ID_VALUE    0x60
#define BME280_RESET_VALUE      0xB6

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_CalibData;

typedef struct {
    int16_t  temperature_x100;
    uint16_t humidity_x100;
    uint32_t pressure_Pa;
} BME280_Data;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    BME280_CalibData   calib;
    int32_t            t_fine;
} BME280_Handle;

HAL_StatusTypeDef BME280_Init(BME280_Handle *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BME280_ReadAll(BME280_Handle *dev, BME280_Data *data);

#endif
