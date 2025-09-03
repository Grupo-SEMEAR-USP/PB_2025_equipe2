#include "h_bridge.h"
#include "encoder.h"
#include "PID.h"
#include "I2C.h"
#include "uart_esp32.h"
#include "pid_ctrl.h"

//Inicializa as filas para troca de dados entre o I2C e o PID
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

void teste_pid()
{
    pid_ctrl_block_handle_t left_pid = init_pid(LEFT_MOTOR);
    pid_ctrl_block_handle_t right_pid = init_pid(RIGHT_MOTOR);

    target_rads_data_t target_rads;

    target_rads.target_left_rads = 50;
    target_rads.target_right_rads = 50;

    int contador = 0;
    
    while(contador != 1500)
    {
    
        pid_calculate(left_pid, LEFT_MOTOR, target_rads.target_left_rads, &valpidL, left_encoder);
        pid_calculate(right_pid, RIGHT_MOTOR, target_rads.target_right_rads, &valpidR, right_encoder);

        /*
        if(contador < 1250)
        {
            target_rpm.target_left_rpm += 0.1592;
            target_rpm.target_right_rpm += 0.1592;
        }
        else
        {
            target_rpm.target_left_rpm = 200;
            target_rpm.target_right_rpm = 200;
        }
        */

        vTaskDelay(pdMS_TO_TICKS(20));

        contador++;
        
    }

    pid_del_control_block(left_pid);
    pid_del_control_block(right_pid);
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
            pid_calculate(left_pid, LEFT_MOTOR, target_rads.target_left_rads, &valpidL, left_encoder);
            pid_calculate(right_pid, RIGHT_MOTOR, target_rads.target_right_rads, &valpidR, right_encoder);

        }
        
        vTaskDelay(pdMS_TO_TICKS(2 * FREQ_COMUNICATION));
    }
}

/*
void i2c_task_com()
{
    
    rpm_data_t last_rpm = {0,0};
    target_rpm_data_t last_target_rpm = {0,0};
    
    ESP_ERROR_CHECK(init_i2c());

    while(1)
    {
        i2c_read_task(last_target_rpm);
        i2c_write_task(last_rpm);
        
        vTaskDelay(pdMS_TO_TICKS(FREQ_COMUNICATION));
    }

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_SLAVE_NUM));
}
*/

void app_main()
{
    init_all();

    rads_queue = xQueueCreate(10, sizeof(rads_data_t));
    target_rads_queue = xQueueCreate(10, sizeof(target_rads_data_t));

    teste_pid();
    
    //xTaskCreatePinnedToCore(task_motor_control, "task_motor_control", 4096, NULL, 1, NULL, 0);
    //xTaskCreatePinnedToCore(i2c_task_com, "i2c_task_com", 4096, NULL, 1, NULL, 1);
    
}





