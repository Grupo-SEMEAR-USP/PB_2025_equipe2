#include "h_bridge.h"
#include "encoder.h"
#include "PID.h"
#include "uart_esp32.h"
#include "pid_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"


#define LOG_LOCAL_LEVEL ESP_LOG_NONE

// Filas para comunicação entre tasks
QueueHandle_t target_rads_queue;

// Handles dos encoders
pcnt_unit_handle_t left_encoder;
pcnt_unit_handle_t right_encoder;

// Variáveis globais
float valpidR = 0, valpidL = 0;
const char* TAG_MAIN = "MAIN_ROBOT";

void init_all()
{
    ESP_LOGI(TAG_MAIN, "Inicializando hardware..."); 
    
    init_gpio(LEFT_MOTOR);
    init_gpio(RIGHT_MOTOR);

    init_pwm(LEFT_MOTOR);
    init_pwm(RIGHT_MOTOR);

    ESP_LOGI(TAG_MAIN, "Encoders..."); 
    left_encoder = init_encoder(ENCODER_LEFT);
    right_encoder = init_encoder(ENCODER_RIGHT);

    ESP_LOGI(TAG_MAIN, "UART..."); 
    init_uart(); 
    
    ESP_LOGI(TAG_MAIN, "Hardware inicializado!"); 
}

// Task 1: Comunicação UART com ROS
void uart_communication_task(void *pvParameters)
{
    target_rads_data_t target_rads_rcv;
    target_rads_data_t last_target_rads = {0, 0};
    rads_data_t current_rads;
    
    // Fator de conversão (pulsos -> rad/s)
    const float conversion_factor = 0.00475 * 20.0; // 0.095
    
    ESP_LOGI(TAG_MAIN, "Task UART iniciada (Loop de 20Hz)"); 
    
    while(1) {
        //Recebe comandos 
        target_rads_rcv = receive_data(&last_target_rads);
        
        //Verifica se o comando e novo
        if (target_rads_rcv.target_left_rads != last_target_rads.target_left_rads ||
            target_rads_rcv.target_right_rads != last_target_rads.target_right_rads) {
            
           //Envia comando novo para a Fila do PID
            xQueueSend(target_rads_queue, &target_rads_rcv, 0);
            ESP_LOGI(TAG_MAIN, "Novo Comando ROS -> Fila: L=%.3f, R=%.3f", 
                    target_rads_rcv.target_left_rads, target_rads_rcv.target_right_rads);
            
            //Atualiza o ultimo comando valido
            last_target_rads = target_rads_rcv;
        }

        // 5. Calcular velocidade atual (Odometria)
        // Certifique-se que pulse_count() zera a contagem após ler
        current_rads.left_rads = (float)pulse_count(left_encoder) * conversion_factor; // rad/s
        current_rads.right_rads = (float)pulse_count(right_encoder) * conversion_factor; // rad/s
        
        // 6. Enviar odometria para o PC
        send_data(current_rads);
        
        vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz 
    }
}

// Task 2: Controle PID dos Motores
void motor_control_task(void *pvParameters)
{
    pid_ctrl_block_handle_t left_pid = init_pid(LEFT_MOTOR);
    pid_ctrl_block_handle_t right_pid = init_pid(RIGHT_MOTOR);
    
    // Armazena o último comando de velocidade recebido
    target_rads_data_t current_target_rads = {0.0, 0.0};

    ESP_LOGI(TAG_MAIN, "Task controle motor iniciada (Loop de 50Hz)"); 
    
    while(1) {
        
        // 1. Verifica se há um novo comando na fila 
        if(xQueueReceive(target_rads_queue, &current_target_rads, 0) == pdPASS) {
            // Novo comando recebido
            ESP_LOGI(TAG_MAIN, "PID Task: Novo alvo recebido L=%.3f, R=%.3f", 
                    current_target_rads.target_left_rads, current_target_rads.target_right_rads);
        }
        
        // 2. Executa o PID continuamente com o último comando recebido
        //se nenhum comando foi recebido pega o ultimo ou 0.0
        ESP_LOGI("Target","%.3f %.3f", current_target_rads.target_right_rads,current_target_rads.target_left_rads);
        pid_calculate(left_pid, LEFT_MOTOR, current_target_rads.target_left_rads, &valpidL, left_encoder);
        pid_calculate(right_pid, RIGHT_MOTOR, current_target_rads.target_right_rads, &valpidR, right_encoder);
        
        
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz (Frequência de controle)
    }
}

// Task 3: Monitoramento (debug)
void monitoring_task(void *pvParameters)
{
    target_rads_data_t target_rads_monitor;
    
    ESP_LOGI(TAG_MAIN, "Task monitoramento iniciada (Loop de 1Hz)"); 
    
    while(1) {
        
        // Tentar pegar último comando da fila (sem remover)
        if(xQueuePeek(target_rads_queue, &target_rads_monitor, 0) == pdPASS) {
            ESP_LOGI(TAG_MAIN, "[MONITOR] Alvo: L=%.3f, R=%.3f | PID Saída: L=%.1f, R=%.1f",
                    target_rads_monitor.target_left_rads, target_rads_monitor.target_right_rads,
                    valpidL, valpidR);
        } else {
            ESP_LOGI(TAG_MAIN, "[MONITOR] Alvo: (Sem novos) | PID Saída: L=%.1f, R=%.1f",
                    valpidL, valpidR);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1Hz
    }
}

void app_main()
{
    ESP_LOGI(TAG_MAIN, "INICIANDO..."); 
    
    // 1. Inicializar hardware
   init_all();
    
    // 2. Criar fila
    target_rads_queue = xQueueCreate(10, sizeof(target_rads_data_t));
    
    if (target_rads_queue == NULL) {
        ESP_LOGE(TAG_MAIN, "Erro criando fila target_rads_queue!");
        return;
    }
    
    // 3. Criar tasks
    xTaskCreatePinnedToCore(uart_communication_task, "uart_comm", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, NULL, 10, NULL, 0); 
    xTaskCreate(monitoring_task, "monitor", 4096, NULL, 1, NULL); 
    
    ESP_LOGI(TAG_MAIN, "Sistema inicializado! Pronto para comunicação ROS.");
    
    // Task principal não faz mais nada
    vTaskSuspend(NULL); 
}