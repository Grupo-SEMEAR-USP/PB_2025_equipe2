#include "h_bridge.h"

//Inicializa os pinos da ESP (motores, PMW, standBy)
void init_gpio(motor_side_t motor)
{

    gpio_set_direction(MOTOR_INPUT_1(motor), GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_INPUT_2(motor), GPIO_MODE_OUTPUT);

    gpio_set_direction(PWM_INPUT(motor), GPIO_MODE_OUTPUT);

    gpio_set_direction(pinStandBy, GPIO_MODE_OUTPUT);
    gpio_set_level(pinStandBy, HIGH);

}

//Configura o PMW (timer e canal)
void init_pwm(motor_side_t motor)
{

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_TIMER_10_BIT, 
        .freq_hz          = 500, 
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t motor_channel = {
        .channel    = MOTOR_CHANNEL(motor),
        .duty       = 0,
        .gpio_num   = PWM_INPUT(motor),
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&motor_channel);
}

//Chaveamento da ponte H para o motor rodar p/ frente
esp_err_t _set_forward(motor_side_t motor)
{
    gpio_set_level(MOTOR_INPUT_1(motor), HIGH);
    gpio_set_level(MOTOR_INPUT_2(motor), LOW);
    return ESP_OK;
}

//Chaveamento da ponte H para o motor rodar p/ trás
esp_err_t _set_backward(motor_side_t motor)
{
    gpio_set_level(MOTOR_INPUT_1(motor), LOW);
    gpio_set_level(MOTOR_INPUT_2(motor), HIGH);
    return ESP_OK;
}

//Atualiza o valor do PMW enviado para o motor e a direção desejada
esp_err_t update_motor(motor_side_t motor, int u)
{
    u > 0 ? _set_forward(motor) : _set_backward(motor);

    u = abs(u);

    if(u > 1023)
        u = 1023;
    
    ledc_set_duty(LEDC_MODE, MOTOR_CHANNEL(motor), u);
    ledc_update_duty(LEDC_MODE, MOTOR_CHANNEL(motor));
    return ESP_OK;
}