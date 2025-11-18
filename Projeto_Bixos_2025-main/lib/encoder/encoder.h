#ifndef ENCODER_H
#define ENCODER_H

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#define PCNT_HIGH_LIMIT     1500
#define PCNT_LOW_LIMIT     -1500

#define ENCODER_CHA_R1      GPIO_NUM_33//AC2
#define ENCODER_CHA_R2      GPIO_NUM_32//AC1

#define ENCODER_CHA_L1      GPIO_NUM_26//BC2
#define ENCODER_CHA_L2      GPIO_NUM_27//BC1

typedef enum
{
    ENCODER_LEFT = 0,
    ENCODER_RIGHT = 1

} encoder_side_t;

#define ENCODER_INPUT_A(NUM) NUM == (ENCODER_LEFT) ? ENCODER_CHA_L1 : ENCODER_CHA_R1
#define ENCODER_INPUT_B(NUM) NUM == (ENCODER_LEFT) ? ENCODER_CHA_L2 : ENCODER_CHA_R2

pcnt_unit_handle_t init_encoder(encoder_side_t encoder);
bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
float pulse_count(pcnt_unit_handle_t encoder);

#endif