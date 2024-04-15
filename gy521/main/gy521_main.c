#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "MPU6050.h"
#include <string.h>
#include <math.h>

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_CLK_FREQ 100000
#define TEST_I2C_PORT -1 // auto-detect

static const char *TAG = "gy521";

static const MPU6050_CALIBRATION_PARAMS mpu6050_nonideal = {
    .accel_offset = {520, 0, 0-1550},
    .accel_gain = {0.9973, 1.0022, 1.0162},
    .temp_offset = 0,
    .temp_gain = 1,
    .gyro_offset = {0, 0, 0},
    .gyro_gain = {1, 1, 1}
};

i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TEST_I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

i2c_master_bus_handle_t bus_handle;

i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDRESS_AD0_LOW,
        .scl_speed_hz = I2C_CLK_FREQ,
    };

i2c_master_dev_handle_t dev_handle;

/**
 * @brief Configure I2C bus for master
 */
static void i2c_config(){
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
}

/**
 * @brief Read a sequence of bytes from a MPU6050 sensor registers
 */
static esp_err_t mpu6050_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    const uint8_t write_buf[1] = {reg_addr};
    return i2c_master_transmit_receive(dev_handle, write_buf, (size_t)1, data, len, -1);
}

/**
 * @brief Write a byte to a MPU6050 sensor register
 */
static esp_err_t mpu6050_write(uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, (size_t)2, -1);
}

/**
 * @brief Read 3-axis acceleration of MPU6050 sensor registers and correct raw values
 */
static double * mpu6050_get_accel()
{
    uint8_t buf[6];
    double *accel = (double *)malloc(3 * sizeof(double));
    ESP_ERROR_CHECK(mpu6050_read(MPU6050_RA_ACCEL_XOUT_H, buf, 6));
    for (int i = 0; i < 3; i++)
    {
        int16_t raw = (int16_t)(buf[2*i] << 8 | buf[2*i+1]); // combine registers
        accel[i] = (double)(raw - mpu6050_nonideal.accel_offset[i]) / mpu6050_nonideal.accel_gain[i] / 16384.0; // adjust to calibration
    }
    return accel;
}

/**
 * @brief Calculate angles from 3-axis accelerometer values
 */
static double * calc_angle(double *accel, uint8_t degree_opt)
{
    double *angle = (double *)malloc(3 * sizeof(double));
    double conversion = degree_opt ? (180.0 / M_PI) : 1;
    angle[0] = atan2(accel[0], sqrt(pow(accel[1],2) + pow(accel[2],2))) * conversion; // theta
    angle[1] = atan2(accel[1], sqrt(pow(accel[0],2) + pow(accel[2],2))) * conversion; // psi
    angle[2] = atan2(sqrt(pow(accel[0],2) + pow(accel[1],2)), accel[2]) * conversion; // phi

    return angle;
}

/**
 * @brief Initialization of MPU6050
 */
static void mpu6050_init()
{
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_RA_PWR_MGMT_1, 0 << MPU6050_PWR1_SLEEP_BIT));
    ESP_LOGI(TAG, "MPU6050: awake");

    ESP_ERROR_CHECK(mpu6050_write(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2 << MPU6050_ACONFIG_AFS_SEL_BIT));
    ESP_LOGI(TAG, "MPU6050: accel FSR --> 2g");
}

void mpu6050_task(){

    mpu6050_init();

    double *accel;
    double *angle;
    uint8_t degree_opt = 1;

    while(1){

        accel = mpu6050_get_accel();
        //printf("[ACCEL]\tX: %.3f\tY: %.3f\tZ: %.3f\r\n", accel[0], accel[1], accel[2]);

        angle = calc_angle(accel, degree_opt);
        printf("[ANGLE]\ttheta: %.f\tpsi: %.f\t\tphi: %.f\r\n", angle[0], angle[1], angle[2]); 

        fflush(stdout);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    i2c_config();
    xTaskCreate(&mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
}