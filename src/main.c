#include "h_bridge.h"
#include "encoder.h"
#include "PID.h"
#include "uart_esp32.h"
#include "pid_ctrl.h"

//Inicializa as filas para troca de dados entre o UART e o PID
QueueHandle_t rads_queue;
QueueHandle_t target_rads_queue;

pcnt_unit_handle_t left_encoder;
pcnt_unit_handle_t right_encoder;

float valpidR = 0, valpidL = 0;

void init_all()
{
    init_gpio(LEFT_MOTOR);
    init_gpio(RIGHT_MOTOR);

    init_pwm(LEFT_MOTOR);
    init_pwm(RIGHT_MOTOR);

    left_encoder = init_encoder(ENCODER_LEFT);
    right_encoder = init_encoder(ENCODER_RIGHT);

}


void task_motor_control()
{
    pid_ctrl_block_handle_t left_pid = init_pid(LEFT_MOTOR);
    pid_ctrl_block_handle_t right_pid = init_pid(RIGHT_MOTOR);

    target_rads_data_t target_rads;

    while(1)
    {
        
        if(xQueueReceive(target_rads_queue, &target_rads, 0) == pdPASS)
        {
            pid_calculate(left_pid, LEFT_MOTOR, target_rads.target_left_rads, &valpidL, left_encoder, 10);
            pid_calculate(right_pid, RIGHT_MOTOR, target_rads.target_right_rads, &valpidR, right_encoder, 10);

        }
        
        vTaskDelay(pdMS_TO_TICKS(FREQ_COMUNICATION));
    }
}

void app_main()
{
    init_all();

    rads_queue = xQueueCreate(10, sizeof(rads_data_t));
    target_rads_queue = xQueueCreate(10, sizeof(target_rads_data_t));

    init_uart_read();

    target_rads_data_t last_target_rads = {0,0};
        
    pid_ctrl_block_handle_t left_pid = init_pid(LEFT_MOTOR);
    pid_ctrl_block_handle_t right_pid = init_pid(RIGHT_MOTOR);

    while(1)
    {
        target_rads_data_t target_rads = receive_data(&last_target_rads);

        pid_calculate(left_pid, LEFT_MOTOR, target_rads.target_left_rads, &valpidR, left_encoder, 10);
        pid_calculate(right_pid, RIGHT_MOTOR, target_rads.target_right_rads, &valpidR, left_encoder, 10);
    }
     
    //xTaskCreatePinnedToCore(task_motor_control, "task_motor_control", 4096, NULL, 1, NULL, 0);
    
}





