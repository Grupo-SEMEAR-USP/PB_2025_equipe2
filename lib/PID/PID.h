#ifndef PID_H
#define PID_H

#include "h_bridge.h"
#include "encoder.h"
#include "esp_err.h"
#include "esp_check.h"
#include "pid_ctrl.h"

#define KP_R 1
#define KI_R 0
#define KD_R 0
#define MAX_OUTPUT_R 1023
#define MIN_OUTPUT_R -1023
#define MAX_INTEGRAL_R 1023
#define MIN_INTEGRAL_R -1023

#define KP_L 1
#define KI_L 0
#define KD_L 0
#define MAX_OUTPUT_L 1023
#define MIN_OUTPUT_L -1023
#define MAX_INTEGRAL_L 1023 
#define MIN_INTEGRAL_L -1023

#define kp(MOTOR) (MOTOR == RIGHT_MOTOR)? KP_R : KP_L
#define ki(MOTOR) (MOTOR == RIGHT_MOTOR)? KI_R : KI_L
#define kd(MOTOR) (MOTOR == RIGHT_MOTOR)? KD_R : KD_L
#define max_output(MOTOR) (MOTOR == RIGHT_MOTOR)? MAX_OUTPUT_R : MAX_OUTPUT_L
#define min_output(MOTOR) (MOTOR == RIGHT_MOTOR)? MIN_OUTPUT_R : MIN_OUTPUT_L
#define max_integral(MOTOR) (MOTOR == RIGHT_MOTOR)? MAX_INTEGRAL_R : MAX_INTEGRAL_L
#define min_integral(MOTOR) (MOTOR == RIGHT_MOTOR)? MIN_INTEGRAL_R : MIN_INTEGRAL_L

#define FREQ_COMUNICATION       10

pid_ctrl_block_handle_t init_pid(motor_side_t motor);

void PWM_limit(float*);

esp_err_t pid_calculate(pid_ctrl_block_handle_t pid, motor_side_t motor, float target_rpm, float* inc_value, pcnt_unit_handle_t encoder);

#endif