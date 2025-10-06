#include "encoder.h"

const char *TAG_ENCODER = "Encoder";

//Configura e inicializa o encoder (retorna o 'pcnt_unit_handle_t' pronto)
pcnt_unit_handle_t init_encoder(encoder_side_t encoder)
{
    pcnt_unit_handle_t selected_encoder = NULL;

    ESP_LOGI(TAG_ENCODER, "Install pcnt unit");
    pcnt_unit_config_t unit_config = {

        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT

    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &selected_encoder));

    ESP_LOGI(TAG_ENCODER, "Set glich filter");
    pcnt_glitch_filter_config_t filter_config = {

        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(selected_encoder, &filter_config));

    ESP_LOGI(TAG_ENCODER, "Install pcnt channels");
    pcnt_chan_config_t channel_a_config = {

        .edge_gpio_num = ENCODER_INPUT_A(encoder),
        .level_gpio_num = ENCODER_INPUT_B(encoder)

    };
    pcnt_channel_handle_t pcnt_channel_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(selected_encoder, &channel_a_config, &pcnt_channel_a));

    pcnt_chan_config_t channel_b_config = {

        .edge_gpio_num = ENCODER_INPUT_B(encoder),
        .level_gpio_num = ENCODER_INPUT_A(encoder)

    };
    pcnt_channel_handle_t pcnt_channel_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(selected_encoder, &channel_b_config, &pcnt_channel_b));

    ESP_LOGI(TAG_ENCODER, "Set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG_ENCODER, "Enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(selected_encoder));

    ESP_LOGI(TAG_ENCODER, "Clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(selected_encoder));

    ESP_LOGI(TAG_ENCODER, "Start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(selected_encoder));

    return selected_encoder;
}

bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;

    xQueueSendFromISR(queue, &(edata ->watch_point_value), &high_task_wakeup);

    return (high_task_wakeup == pdTRUE);
}

//Retorna os pulsos contados
float pulse_count(pcnt_unit_handle_t encoder)
{
    int pulse_count = 0;

    pcnt_unit_clear_count(encoder);
    
    ESP_ERROR_CHECK(pcnt_unit_get_count(encoder, &pulse_count));
    ESP_LOGI(TAG_ENCODER, "%f", pulse_count*0.00475);

    vTaskDelay(pdMS_TO_TICKS(10));
    
    return pulse_count;
}

