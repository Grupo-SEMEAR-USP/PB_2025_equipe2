#include "uart_esp32.h"
#include "h_bridge.h"
#include "PID.h"
#include "encoder.h"
#include "esp_log.h"
#include <string.h> // Para strtok_r
#include <stdlib.h> // Para atof

#define LOG_LOCAL_LEVEL ESP_LOG_NONE

static const char *UART_TAG = "UART_ESP32";

void init_uart()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT, // Adicionado para clareza
    };

    // Usar a MESMA porta para ler e escrever
    ESP_LOGI(UART_TAG, "Configurando UART %d", UART_PORT_NUM);

    // Instalar driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUFFER_LEN * 2, 0, 0, NULL, 0));
    // Configurar parâmetros
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    // Configurar pinos
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(UART_TAG, "UART %d configurada nos pinos TX:%d, RX:%d", UART_PORT_NUM, TXD_PIN, RXD_PIN);
}

target_rads_data_t receive_data(target_rads_data_t *last_target_rads)
{
    target_rads_data_t target_rads = *last_target_rads; // Usar último valor como padrao

    uint8_t buffer[BUFFER_LEN];
    int len = uart_read_bytes(UART_PORT_NUM, buffer, BUFFER_LEN - 1, pdMS_TO_TICKS(20)); 

    if(len > 0)
    {
        buffer[len] = '\0';
        ESP_LOGD(UART_TAG, "Recebido RAW: %s", buffer); 

        char *saveptr; // Para strtok_r
        char *token1 = strtok_r((char *)buffer, ";", &saveptr);
        char *token2 = strtok_r(NULL, "\n", &saveptr); // Buscar até ao \n

        if(token1 != NULL && token2 != NULL)
        {
            float new_left = atof(token1);
            float new_right = atof(token2);

            target_rads.target_left_rads = new_left;
            target_rads.target_right_rads = new_right;

            
            ESP_LOGI(UART_TAG, "Comando processado: L=%.3f, R=%.3f", 
                    target_rads.target_left_rads, target_rads.target_right_rads);
        }
        else
        {
            ESP_LOGW(UART_TAG, "Formato inválido recebido: %s", (char*)buffer);
        }
    }

    return target_rads;
}


void send_data(rads_data_t rads)
{
    char buffer[BUFFER_LEN];
    
    //Formato ->"esquerda;direita\n"
    int length = snprintf(buffer, sizeof(buffer), "%.3f;%.3f\n", 
                         rads.left_rads, rads.right_rads);
    
    if (length > 0 && length < sizeof(buffer)) {
        uart_write_bytes(UART_PORT_NUM, buffer, length); 
        ESP_LOGD(UART_TAG, "Enviado: %s", buffer); 
    } else {
        ESP_LOGE(UART_TAG, "Erro ao formatar dados para envio");
    }
}