#include "I2C.h"

//Inicializa e configura o dispositivo como Servo
int init_i2c()
{
    i2c_config_t config = {

        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 0

    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_SLAVE_NUM, &config));
    return i2c_driver_install(I2C_SLAVE_NUM, config.mode, I2C_SLAVE_RX_BUF, I2C_SLAVE_TX_BUF, 0);
}

//Recebe o valor de RPM visado (pós tratamento) e manda para o PID
void i2c_read_task(target_rpm_data_t last_target_rpm) 
{
    target_rpm_data_t target_rpm;
    uint8_t data[8];

    //Recebe da Rasp
    int buffer = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, sizeof(data), 10);
    if(buffer == sizeof(data))
    {
        last_target_rpm = target_rpm;
    }

    else if(buffer > 0 && buffer < sizeof(data))
    {
        target_rpm = last_target_rpm;
        ESP_LOGW("I2C", "Erro na leitura | %d bytes lidos", buffer);

    }

    else if(buffer == 0)
    {
        ESP_LOGW("I2C", "Nenhum byte lido, reiniciando I2C...");
        //reset_i2c(I2C_SLAVE_NUM);
    }

    memcpy(&target_rpm.target_left_rpm, &data[0], 4);
    memcpy(&target_rpm.target_right_rpm, &data[4], 4);
        
    //Manda pro PID
    xQueueOverwrite(target_rpm_queue, &target_rpm);

}

//Lê dados atuais de RPM e manda pra Rasp
void i2c_write_task(rpm_data_t last_rpm)
{
    rpm_data_t rpm;
    uint8_t data[8];

    //Tenta ler dos encoders (PID)
    if(xQueueReceive(rpm_queue, &rpm, 0) == pdPASS)
    {
        last_rpm = rpm;
    }

    else 
    {
        rpm = last_rpm;
        ESP_LOGW("I2C", "Erro na leitura de fila de RPM");
    }

    memcpy(&data[0], &rpm.left_rpm, 4);
    memcpy(&data[4], &rpm.right_rpm, 4);

    //Manda pra Rasp
    int bytes_written = i2c_slave_write_buffer(I2C_SLAVE_NUM, data, sizeof(data), pdMS_TO_TICKS(FREQ_COMUNICATION));

    if(bytes_written != sizeof(data))
    {
        ESP_LOGW("I2C", "Falha ao escrever, %d enviados", bytes_written);
    }

    

}