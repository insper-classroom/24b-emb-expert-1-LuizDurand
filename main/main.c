/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// main.c

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <math.h> 

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include <Fusion.h>

// Definições
const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

// Estrutura para comunicação via fila
typedef struct Mouse {
    int axis;
    int val;
} mouse;

// Handler para a fila
QueueHandle_t xQueueAdc;

// Task para ler dados do MPU6050
static void mpu6050_task(void *p) {
    imu_c imu_config;
    mouse mouse_data;

    // Configura o driver MPU6050
    mpu6050_set_config(&imu_config, i2c0, I2C_SDA_GPIO, I2C_SCL_GPIO, 0); // acc_scale = 0 para ±2G

    if (!mpu6050_init(imu_config)) {
        
        vTaskDelete(NULL);
    }

    // Reinicia o MPU6050
    if (!mpu6050_reset(imu_config)) {
        
        vTaskDelete(NULL);
    }

    // Configuração opcional de detecção de movimento
    mpu6050_set_motion_detection_threshold(imu_config, 1);
    mpu6050_set_motion_detection_duration(imu_config, 20);
    mpu6050_set_motion_detection(imu_config, 1);

    int16_t acceleration[3], gyro[3], temp;

    // Inicializa o FusionAhrs
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    // Define o período de amostragem
    #define SAMPLE_PERIOD (0.01f) // 10 ms

    while(1) {
        if (!mpu6050_read_acc(imu_config, acceleration) ||
            !mpu6050_read_gyro(imu_config, gyro) ||
            !mpu6050_read_temp(imu_config, &temp)) {
            
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Converte os dados brutos para unidades físicas
        FusionVector gyroscope;
        gyroscope.axis.x = gyro[0] / 131.0f;   // Converter para graus/s
        gyroscope.axis.y = gyro[1] / 131.0f;
        gyroscope.axis.z = gyro[2] / 131.0f;

        FusionVector accelerometer;
        accelerometer.axis.x = acceleration[0] / 16384.0f;  // Converter para g
        accelerometer.axis.y = acceleration[1] / 16384.0f;
        accelerometer.axis.z = acceleration[2] / 16384.0f;

        // Atualiza o FusionAhrs sem dados do magnetômetro
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        // Obtém os ângulos de Euler a partir do quaternion
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Verifica os thresholds e envia dados para a fila
        if(fabs(euler.angle.roll) >= 10){
            mouse_data.axis = 1;
            mouse_data.val = -euler.angle.roll;
            xQueueSend(xQueueAdc, &mouse_data, portMAX_DELAY);
        }
        if(fabs(euler.angle.yaw) >= 10){
            mouse_data.axis = 0;
            mouse_data.val = -euler.angle.yaw;
            xQueueSend(xQueueAdc, &mouse_data, portMAX_DELAY);
        }
        if(fabs(accelerometer.axis.y) >= 1.5){
            mouse_data.axis = 2;
            mouse_data.val = 1;
            xQueueSend(xQueueAdc, &mouse_data, portMAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Task para enviar dados via UART
void uart_task(void *p) {
    mouse mouse_data;

    while (1) {
        if(xQueueReceive(xQueueAdc, &mouse_data, portMAX_DELAY)){
            uart_putc(uart0, mouse_data.axis);
            uart_putc(uart0, (mouse_data.val >> 8) & 0xFF);
            uart_putc(uart0, mouse_data.val & 0xFF);
            uart_putc(uart0, -1); // Delimitador de pacote
        }
    }
}

int main() {
    // Inicializa todos os periféricos padrão
    stdio_init_all();

    // Configura UART0
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    // Cria a fila para comunicação entre tasks
    xQueueAdc = xQueueCreate(32, sizeof(mouse));

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 1, NULL);

    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();

    // Loop infinito caso o scheduler falhe
    while (true)
        ;
}
