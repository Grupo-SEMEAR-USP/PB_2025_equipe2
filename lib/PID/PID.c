#include "PID.h"
#include "pid_ctrl.h"

const char *TAG_PID = "PID";
const char *TAG_PID_CTRL = "PID CTRL";

//Inicializa e configura o PID (retorna o 'pid_ctrl_block_handle_t' pronto)
pid_ctrl_block_handle_t init_pid(motor_side_t motor)
{

    pid_ctrl_parameter_t pid_param = {

        .kp = kp(motor),
        .ki = ki(motor),
        .kd = kd(motor),

        .max_integral = max_integral(motor),
        .min_integral = min_integral(motor),

        .max_output = max_output(motor),
        .min_output = min_output(motor),

        .cal_type = PID_CAL_TYPE_INCREMENTAL
    };

    pid_ctrl_config_t pid_ctrl_config = {

        .init_param = pid_param
    };

    pid_ctrl_block_handle_t pid;

    ESP_ERROR_CHECK(pid_new_control_block(&pid_ctrl_config, &pid));
    return pid;
}

float speed_conversion(float time, pcnt_unit_handle_t encoder)
{
    float conversion_rate = 0.00475;
    float speed = pulse_count(encoder) * conversion_rate/pdMS_TO_TICKS(time) * 1000;

    return speed;
}

//Aplica o controle PID sobre o erro da velocidade, atualizando o PMW dos motores
esp_err_t pid_calculate(pid_ctrl_block_handle_t pid, motor_side_t motor, float target_rads, float* inc_value, pcnt_unit_handle_t encoder, float time)
{

    float current_rads = speed_conversion(time, encoder);

    float error = target_rads - current_rads;

    float value;

    ESP_ERROR_CHECK(pid_compute(pid, error, &value));

    *inc_value += value;

    update_motor(motor, *inc_value);

    ESP_LOGI(TAG_PID, "Alvo: %f, | Erro: %f, | PID: %f\n", target_rads, error, *inc_value);

    return ESP_OK;
}
