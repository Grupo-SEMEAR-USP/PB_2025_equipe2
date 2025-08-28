#ifndef H_BRIDGE_H
#define H_BRIDGE_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 

#define HIGH    1
#define LOW     0

#define pinStandBy          GPIO_NUM_32

#define INPUT_RIGHT_1       GPIO_NUM_10
#define INPUT_RIGHT_2       GPIO_NUM_15

#define INPUT_LEFT_1        GPIO_NUM_25
#define INPUT_LEFT_2        GPIO_NUM_33

#define PWM_GPIO_LEFT       GPIO_NUM_26
#define PWM_GPIO_RIGHT      GPIO_NUM_20

#define LEDC_CHANNEL_RIGHT   LEDC_CHANNEL_0
#define LEDC_CHANNEL_LEFT    LEDC_CHANNEL_1


#define MOTOR_INPUT_1(MOTOR) (MOTOR == (LEFT_MOTOR)) ? INPUT_LEFT_1 : INPUT_RIGHT_1
#define MOTOR_INPUT_2(MOTOR) (MOTOR == (LEFT_MOTOR)) ? INPUT_LEFT_2 : INPUT_RIGHT_2

#define PWM_INPUT(MOTOR) (MOTOR == (LEFT_MOTOR)) ? PWM_GPIO_LEFT : PWM_GPIO_RIGHT

#define MOTOR_CHANNEL(MOTOR) (MOTOR == (LEFT_MOTOR))? LEDC_CHANNEL_LEFT : LEDC_CHANNEL_RIGHT


typedef enum {

    LEFT_MOTOR,
    RIGHT_MOTOR,

} motor_side_t;


#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0


void init_gpio(motor_side_t motor);
void init_pwm(motor_side_t motor);
esp_err_t _set_forward(motor_side_t motor);
esp_err_t _set_backward(motor_side_t motor);
esp_err_t update_motor(motor_side_t motor, int u);

#endif