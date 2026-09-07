#include "main.h"
#include "bme280.h"
#include "can_comms.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern CAN_HandleTypeDef hcan1;
extern UART_HandleTypeDef huart2;

#define FAN_RELAY_PIN       GPIO_PIN_0
#define FAN_RELAY_PORT      GPIOB
#define STATUS_LED_PIN      GPIO_PIN_5
#define STATUS_LED_PORT     GPIOA

#define TEMP_THRESHOLD_ON   2800
#define TEMP_THRESHOLD_OFF  2600

#define SENSOR_READ_INTERVAL_MS     250
#define CAN_BROADCAST_INTERVAL_MS   500
#define HEARTBEAT_INTERVAL_MS       1000
#define LED_BLINK_INTERVAL_MS       500

static BME280_Handle bme280;
static BME280_Data   sensor_data;
static uint8_t       fan_state = FAN_STATE_OFF;

static void fan_control_update(void)
{
    if (fan_state == FAN_STATE_OFF) {
        if (sensor_data.temperature_x100 > TEMP_THRESHOLD_ON) {
            fan_state = FAN_STATE_ON;
            HAL_GPIO_WritePin(FAN_RELAY_PORT, FAN_RELAY_PIN, GPIO_PIN_SET);
        }
    } else {
        if (sensor_data.temperature_x100 < TEMP_THRESHOLD_OFF) {
            fan_state = FAN_STATE_OFF;
            HAL_GPIO_WritePin(FAN_RELAY_PORT, FAN_RELAY_PIN, GPIO_PIN_RESET);
        }
    }
}

int _write(int file, char *data, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100);
    return len;
}

void app_init(void)
{
    printf("\r\n=== ECU starting ===\r\n");

    if (BME280_Init(&bme280, &hi2c1) != HAL_OK) {
        printf("ERROR: BME280 init failed\r\n");
        while (1) {
            HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
            HAL_Delay(100);
        }
    }
    printf("BME280 OK\r\n");

    if (CAN_Comms_Init(&hcan1) != HAL_OK) {
        printf("ERROR: CAN init failed\r\n");
        while (1) {
            HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
            HAL_Delay(100);
        }
    }
    printf("CAN OK (500 kbps)\r\n");

    HAL_GPIO_WritePin(FAN_RELAY_PORT, FAN_RELAY_PIN, GPIO_PIN_RESET);
    fan_state = FAN_STATE_OFF;

    printf("Init complete. ON > %d.%02d C, OFF < %d.%02d C\r\n",
           TEMP_THRESHOLD_ON / 100, TEMP_THRESHOLD_ON % 100,
           TEMP_THRESHOLD_OFF / 100, TEMP_THRESHOLD_OFF % 100);
}

void app_loop(void)
{
    static uint32_t last_sensor_read  = 0;
    static uint32_t last_can_send     = 0;
    static uint32_t last_heartbeat    = 0;
    static uint32_t last_led_toggle   = 0;

    uint32_t now = HAL_GetTick();

    if ((now - last_sensor_read) >= SENSOR_READ_INTERVAL_MS) {
        last_sensor_read = now;

        if (BME280_ReadAll(&bme280, &sensor_data) == HAL_OK) {
            fan_control_update();

            printf("T=%d.%02d C  H=%d.%02d %%  P=%lu Pa  FAN=%s\r\n",
                   sensor_data.temperature_x100 / 100,
                   sensor_data.temperature_x100 % 100,
                   sensor_data.humidity_x100 / 100,
                   sensor_data.humidity_x100 % 100,
                   (unsigned long)sensor_data.pressure_Pa,
                   fan_state ? "ON" : "OFF");
        } else {
            printf("WARN: BME280 read failed\r\n");
        }
    }

    if ((now - last_can_send) >= CAN_BROADCAST_INTERVAL_MS) {
        last_can_send = now;
        CAN_SendSensorData(&hcan1, &sensor_data, fan_state);
    }

    if ((now - last_heartbeat) >= HEARTBEAT_INTERVAL_MS) {
        last_heartbeat = now;
        CAN_SendHeartbeat(&hcan1, now / 1000);
    }

    if ((now - last_led_toggle) >= LED_BLINK_INTERVAL_MS) {
        last_led_toggle = now;
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
    }
}
