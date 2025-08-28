#ifndef I2C_H
#define I2C_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "encoder.h"
#include "PID.h"

#define I2C_SLAVE_SCL_IO           GPIO_NUM_22
#define I2C_SLAVE_SDA_IO           GPIO_NUM_21

#define I2C_SLAVE_NUM              I2C_NUM_0
#define I2C_SLAVE_FREQ_HZ          100000
#define I2C_SLAVE_TX_BUF           24
#define I2C_SLAVE_RX_BUF           24
#define I2C_SLAVE_TIMEOUT_MS       1000

extern QueueHandle_t target_rpm_queue;
extern QueueHandle_t rpm_queue;

typedef struct {

    float left_rpm;
    float right_rpm;

}rpm_data_t;

typedef struct {

    float target_left_rpm;
    float target_right_rpm;

}target_rpm_data_t;

int init_i2c();
void i2c_read_task(target_rpm_data_t last_target_rpm);
void i2c_write_task(rpm_data_t last_rpm);
int reset_i2c(int i2c);

#endif