#include "uart_esp32.h"
#include "h_bridge.h"
#include "PID.h"
#include "encoder.h"

// static const char *UART_TAG = "UART_ESP32";

void init_uart_read()
{
    const uart_config_t uart_config = {

        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT_NUM_READ, &uart_config);
    uart_set_pin(UART_PORT_NUM_READ, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM_READ, BUFFER_LEN, 0, 0, 0, 0);
}

target_rads_data_t receive_data(target_rads_data_t *last_target_rads)
{
    target_rads_data_t target_rads;

    uint8_t buffer[BUFFER_LEN];

    int len = uart_read_bytes(UART_PORT_NUM_READ, buffer, BUFFER_LEN - 1, pdMS_TO_TICKS(100));

    if(len>0)
    {
        buffer[len] = '\0';

        char *token = strtok((char *)buffer, ";");
        if(token != NULL)
        {
            target_rads.target_left_rads = atof(token);
        }
        
        token = strtok(NULL, ",");
        if(token != NULL)
        {
            target_rads.target_right_rads = atof(token);
        }

        *last_target_rads = target_rads;
        return target_rads;
    }

    return *last_target_rads;
}


void init_uart_write()
{
    const uart_config_t uart_config = {

        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT_NUM_WRITE, &uart_config);
    uart_set_pin(UART_PORT_NUM_WRITE, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM_WRITE, BUFFER_LEN, 0, 0, 0, 0);
}

void send_data(rads_data_t rads)
{
    uint8_t buffer[BUFFER_LEN];

    snprintf((char *)buffer, sizeof(buffer), "%f;%f", rads.left_rads, rads.right_rads);

    uart_write_bytes(UART_PORT_NUM_WRITE, (const char *)buffer, BUFFER_LEN - 1);
}


// int teste_uart_esp()
// {
    //teste 1 - recebe uma string

    // uint8_t frase[50];

    // int len = uart_read_bytes(UART_PORT_NUM_READ, frase, sizeof(frase) -1, pdMS_TO_TICKS(100));

    // ESP_LOGI(UART_TAG, "%d", len);

    // if(len>0)
    // {
    //     frase[len] = '\0';
    //     ESP_LOGI(UART_TAG, "%s", frase);
    // }

// }
