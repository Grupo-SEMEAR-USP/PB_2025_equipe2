#ifndef UART_ESP32_H
#define UART_ESP32_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

// Usando a UART_NUM_0 (USB Serial) e os pinos 1 e 3, como no seu arquivo antigo.
#define UART_PORT_NUM           (UART_NUM_0)
#define TXD_PIN                 (GPIO_NUM_1)
#define RXD_PIN                 (GPIO_NUM_3)

#define BUFFER_LEN              1024 

typedef struct {
    float left_rads;
    float right_rads;
} rads_data_t;

typedef struct {
    float target_left_rads;
    float target_right_rads;
} target_rads_data_t;


void init_uart();

target_rads_data_t receive_data(target_rads_data_t *last_target_rads);
void send_data(rads_data_t rads);


#endif // UART_ESP32_H