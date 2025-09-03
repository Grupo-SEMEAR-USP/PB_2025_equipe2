#include "uart_esp32.h"

static const char *UART_TAG = "UART_ESP32";

void init_uart_read(QueueHandle_t queue)
{
    const uart_config_t uart_config = {

        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT_NUM_READ, &uart_config);
    uart_set_pin(UART_PORT_NUM_READ, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM_READ, BUFFER_LEN, 0, 0, &queue, 0);
}

uint8_t crc_calc(uint8_t *data, int len)
{
    uint8_t crc = 0;
    
    for(int i = 0; i < len; i++)
    {
        crc ^= data[i];
    }

    return crc;
}

void receive_data()
{
    target_rads_data_t target_rads;

    uint8_t data[9];

    int len = uart_read_bytes(UART_PORT_NUM_READ, data, BUFFER_LEN - 1, pdMS_TO_TICKS(100));

    if(len == 9 && data[0] == 0xAA)
    {
        uint8_t crc_received = crc_calc(&data[1], 8);

        if(crc_received == data[9])
        {
            memcpy(&target_rads.target_left_rads, &data[1], sizeof(float));
            memcpy(&target_rads.target_right_rads, &data[5], sizeof(float));

            ESP_LOGI(UART_TAG, "Target E: %f | Target D: %f", target_rads.target_left_rads, target_rads.target_right_rads);
        }
        else
        {
            ESP_LOGW(UART_TAG, "CRC inválido. Desejado: %u | Recebido: %u", data[9], crc_received);
        }
    }

}


void init_uart_write(QueueHandle_t queue)
{
    const uart_config_t uart_config = {

        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_PORT_NUM_WRITE, &uart_config);
    uart_set_pin(UART_PORT_NUM_WRITE, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT_NUM_WRITE, BUFFER_LEN, 0, 0, &queue, 0);
}

void send_data(float left_rads, float right_rads)
{
    rads_data_t rads;
    rads.left_rads = left_rads;
    rads.right_rads= right_rads;

    uint8_t data[9];
    data[1] = 0xAA;

    memcpy(&data[1], &rads.left_rads, sizeof(float));
    memcpy(&data[5], &rads.right_rads, sizeof(float));

    data[9] = crc_calc(&data[1], 8);

    uart_write_bytes(UART_PORT_NUM_WRITE, (const char *)data, sizeof(data));
}