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

#define BUFFER_LEN              1024

typedef struct {

    float left_rads;
    float right_rads;

}rads_data_t;

typedef struct {

    float target_left_rads;
    float target_right_rads;

}target_rads_data_t;

void init_uart_read();
target_rads_data_t receive_data(target_rads_data_t *last_target_rads);

void init_uart_write();
void send_data(rads_data_t rads);

int teste_uart_esp();

#endif