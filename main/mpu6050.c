#include "mpu6050.h"



// Função para configurar a struct de configuração
void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale) {
    config->i2c = i2c;
    config->pin_sda = pin_sda;
    config->pin_scl = pin_scl;
    config->acc_scale = acc_scale;
}

// Função de inicialização do MPU6050
int mpu6050_init(imu_c config) {
    // Inicializa o I2C com a frequência desejada
    i2c_init(config.i2c, 400 * 1000); // 400 kHz

    // Configura os pinos SDA e SCL para função I2C
    gpio_set_function(config.pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(config.pin_scl, GPIO_FUNC_I2C);

    // Ativa os resistores de pull-up
    gpio_pull_up(config.pin_sda);
    gpio_pull_up(config.pin_scl);

    // Verifica se o dispositivo está respondendo corretamente
    uint8_t reg = 0x75; // WHO_AM_I register
    uint8_t data;
    int ret = i2c_write_blocking(config.i2c, MPU6050_ADDRESS, &reg, 1, true);
    if (ret != 1) return 0;

    ret = i2c_read_blocking(config.i2c, MPU6050_ADDRESS, &data, 1, false);
    if (ret != 1 || data != 0x68) return 0; // O valor esperado do WHO_AM_I é 0x68

    // Acorda o sensor (retira do modo sleep)
    uint8_t buf[] = {0x6B, 0x00}; // PWR_MGMT_1 register
    ret = i2c_write_blocking(config.i2c, MPU6050_ADDRESS, buf, 2, false);
    if (ret != 2) return 0;

    return 1;
}

// Função para reiniciar o dispositivo
int mpu6050_reset(imu_c config) {
    uint8_t reset_command[] = {0x6B, 0x80}; // Reset command no PWR_MGMT_1
    int ret = i2c_write_blocking(config.i2c, MPU6050_ADDRESS, reset_command, 2, false);
    if (ret != 2) return 0;

    sleep_ms(100); // Aguarda o reset

    // Retorna o sensor para o modo normal
    uint8_t buf[] = {0x6B, 0x00};
    ret = i2c_write_blocking(config.i2c, MPU6050_ADDRESS, buf, 2, false);
    return (ret == 2) ? 1 : 0;
}

// Função para ler o acelerômetro
int mpu6050_read_acc(imu_c config, int16_t accel[3]) {
    uint8_t buffer[6];
    uint8_t reg = 0x3B; // ACCEL_XOUT_H

    if (i2c_write_blocking(config.i2c, MPU6050_ADDRESS, &reg, 1, true) != 1 ||
        i2c_read_blocking(config.i2c, MPU6050_ADDRESS, buffer, 6, false) != 6) {
        return 0;
    }

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    return 1;
}

// Função para ler o giroscópio
int mpu6050_read_gyro(imu_c config, int16_t gyro[3]) {
    uint8_t buffer[6];
    uint8_t reg = 0x43; // GYRO_XOUT_H

    if (i2c_write_blocking(config.i2c, MPU6050_ADDRESS, &reg, 1, true) != 1 ||
        i2c_read_blocking(config.i2c, MPU6050_ADDRESS, buffer, 6, false) != 6) {
        return 0;
    }

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    return 1;
}

// Função para ler a temperatura
int mpu6050_read_temp(imu_c config, int16_t *temp) {
    uint8_t buffer[2];
    uint8_t reg = 0x41; // TEMP_OUT_H

    if (i2c_write_blocking(config.i2c, MPU6050_ADDRESS, &reg, 1, true) != 1 ||
        i2c_read_blocking(config.i2c, MPU6050_ADDRESS, buffer, 2, false) != 2) {
        return 0;
    }

    *temp = (buffer[0] << 8) | buffer[1];
    return 1;
}

// Função opcional para ativar a detecção de movimento
int mpu6050_set_motion_detection(imu_c config, int enable) {
    uint8_t command[] = {0x38, enable ? 0x40 : 0x00}; // INT_ENABLE register
    return (i2c_write_blocking(config.i2c, MPU6050_ADDRESS, command, 2, false) == 2) ? 1 : 0;
}

// Função para obter o status da detecção de movimento
int mpu6050_get_motion_interrupt_status(imu_c config) {
    uint8_t reg = 0x3A; // INT_STATUS register
    uint8_t status;

    if (i2c_write_blocking(config.i2c, MPU6050_ADDRESS, &reg, 1, true) != 1 ||
        i2c_read_blocking(config.i2c, MPU6050_ADDRESS, &status, 1, false) != 1) {
        return 0;
    }

    return (status & 0x40) ? 1 : 0;
}

// Função para configurar o threshold de detecção de movimento
int mpu6050_set_motion_detection_threshold(imu_c config, uint8_t thr) {
    uint8_t command[] = {0x1F, thr}; // MOT_THR register
    return (i2c_write_blocking(config.i2c, MPU6050_ADDRESS, command, 2, false) == 2) ? 1 : 0;
}

// Função para configurar a duração da detecção de movimento
int mpu6050_set_motion_detection_duration(imu_c config, uint8_t dur) {
    uint8_t command[] = {0x20, dur}; // MOT_DUR register
    return (i2c_write_blocking(config.i2c, MPU6050_ADDRESS, command, 2, false) == 2) ? 1 : 0;
}
