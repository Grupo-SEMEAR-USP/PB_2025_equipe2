#ifndef UART_ESP
#define UART_ESP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

#define TXD_PIN                 1
#define RXD_PIN                 3
#define UART_PORT_NUM_READ      UART_NUM_0
#define UART_PORT_NUM_WRITE     UART_NUM_1

#define BUFFER_LEN      1024

typedef struct {

    float left_rpm;
    float right_rpm;

}rpm_data_t;

typedef struct {

    float target_left_rpm;
    float target_right_rpm;

}target_rpm_data_t;

void init_uart_read(QueueHandle_t queue);
uint8_t crc_calc(uint8_t *data, int len);
void receive_data();
void init_uart_write(QueueHandle_t queue);
void send_data(float left_rpm, float right_rpm);

#endif