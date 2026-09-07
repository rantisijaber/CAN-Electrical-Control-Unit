#include "bme280.h"

static HAL_StatusTypeDef bme280_write_reg(BME280_Handle *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return HAL_I2C_Master_Transmit(dev->hi2c, BME280_I2C_ADDR, buf, 2, 100);
}

static HAL_StatusTypeDef bme280_read_regs(BME280_Handle *dev, uint8_t reg,
                                          uint8_t *dst, uint16_t len)
{
    HAL_StatusTypeDef st;
    st = HAL_I2C_Master_Transmit(dev->hi2c, BME280_I2C_ADDR, &reg, 1, 100);
    if (st != HAL_OK) return st;
    return HAL_I2C_Master_Receive(dev->hi2c, BME280_I2C_ADDR, dst, len, 100);
}

static HAL_StatusTypeDef bme280_read_calibration(BME280_Handle *dev)
{
    HAL_StatusTypeDef st;
    uint8_t buf[26];
    BME280_CalibData *c = &dev->calib;

    st = bme280_read_regs(dev, BME280_REG_CALIB_00, buf, 26);
    if (st != HAL_OK) return st;

    c->dig_T1 = (uint16_t)(buf[1]  << 8) | buf[0];
    c->dig_T2 = (int16_t) ((buf[3]  << 8) | buf[2]);
    c->dig_T3 = (int16_t) ((buf[5]  << 8) | buf[4]);
    c->dig_P1 = (uint16_t)(buf[7]  << 8) | buf[6];
    c->dig_P2 = (int16_t) ((buf[9]  << 8) | buf[8]);
    c->dig_P3 = (int16_t) ((buf[11] << 8) | buf[10]);
    c->dig_P4 = (int16_t) ((buf[13] << 8) | buf[12]);
    c->dig_P5 = (int16_t) ((buf[15] << 8) | buf[14]);
    c->dig_P6 = (int16_t) ((buf[17] << 8) | buf[16]);
    c->dig_P7 = (int16_t) ((buf[19] << 8) | buf[18]);
    c->dig_P8 = (int16_t) ((buf[21] << 8) | buf[20]);
    c->dig_P9 = (int16_t) ((buf[23] << 8) | buf[22]);
    c->dig_H1 = buf[25];

    uint8_t hbuf[7];
    st = bme280_read_regs(dev, BME280_REG_CALIB_26, hbuf, 7);
    if (st != HAL_OK) return st;

    c->dig_H2 = (int16_t) ((hbuf[1] << 8) | hbuf[0]);
    c->dig_H3 = hbuf[2];
    c->dig_H4 = (int16_t) ((hbuf[3] << 4) | (hbuf[4] & 0x0F));
    c->dig_H5 = (int16_t) ((hbuf[5] << 4) | (hbuf[4] >> 4));
    c->dig_H6 = (int8_t)  hbuf[6];

    return HAL_OK;
}

static int32_t bme280_compensate_temp(BME280_Handle *dev, int32_t adc_T)
{
    BME280_CalibData *c = &dev->calib;
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)c->dig_T1 << 1))) * ((int32_t)c->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)c->dig_T1)) *
              ((adc_T >> 4) - ((int32_t)c->dig_T1))) >> 12) *
            ((int32_t)c->dig_T3)) >> 14;

    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t bme280_compensate_press(BME280_Handle *dev, int32_t adc_P)
{
    BME280_CalibData *c = &dev->calib;
    int64_t var1, var2, p;

    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)c->dig_P6;
    var2 = var2 + ((var1 * (int64_t)c->dig_P5) << 17);
    var2 = var2 + (((int64_t)c->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)c->dig_P3) >> 8) +
           ((var1 * (int64_t)c->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)c->dig_P1) >> 33;

    if (var1 == 0) return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)c->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)c->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)c->dig_P7) << 4);

    return (uint32_t)(p >> 8);
}

static uint32_t bme280_compensate_hum(BME280_Handle *dev, int32_t adc_H)
{
    BME280_CalibData *c = &dev->calib;
    int32_t v_x1_u32r;

    v_x1_u32r = dev->t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)c->dig_H4) << 20) -
                   (((int32_t)c->dig_H5) * v_x1_u32r)) +
                  ((int32_t)16384)) >> 15) *
                (((((((v_x1_u32r * ((int32_t)c->dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)c->dig_H3)) >> 11) +
                     ((int32_t)32768))) >> 10) +
                  ((int32_t)2097152)) * ((int32_t)c->dig_H2) + 8192) >> 14));

    v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)c->dig_H1)) >> 4);

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

    uint32_t hum_raw = (uint32_t)(v_x1_u32r >> 12);
    return (hum_raw * 100) >> 10;
}

HAL_StatusTypeDef BME280_Init(BME280_Handle *dev, I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef st;
    dev->hi2c = hi2c;

    uint8_t chip_id = 0;
    st = bme280_read_regs(dev, BME280_REG_CHIP_ID, &chip_id, 1);
    if (st != HAL_OK)                    return st;
    if (chip_id != BME280_CHIP_ID_VALUE) return HAL_ERROR;

    st = bme280_write_reg(dev, BME280_REG_RESET, BME280_RESET_VALUE);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    st = bme280_read_calibration(dev);
    if (st != HAL_OK) return st;

    st = bme280_write_reg(dev, BME280_REG_CTRL_HUM,  0x01);
    if (st != HAL_OK) return st;
    st = bme280_write_reg(dev, BME280_REG_CONFIG,     0x80);
    if (st != HAL_OK) return st;
    st = bme280_write_reg(dev, BME280_REG_CTRL_MEAS,  0x27);
    if (st != HAL_OK) return st;

    return HAL_OK;
}

HAL_StatusTypeDef BME280_ReadAll(BME280_Handle *dev, BME280_Data *data)
{
    HAL_StatusTypeDef st;
    uint8_t raw[8];

    st = bme280_read_regs(dev, BME280_REG_PRESS_MSB, raw, 8);
    if (st != HAL_OK) return st;

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8)  |  (int32_t)raw[7];

    data->temperature_x100 = (int16_t)bme280_compensate_temp(dev, adc_T);
    data->pressure_Pa      = bme280_compensate_press(dev, adc_P);
    data->humidity_x100    = (uint16_t)bme280_compensate_hum(dev, adc_H);

    return HAL_OK;
}
