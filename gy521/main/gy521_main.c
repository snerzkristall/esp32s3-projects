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
static void mpu6050_get_accel(double *accel, int16_t fsr)
{
    uint8_t buf[6];
    ESP_ERROR_CHECK(mpu6050_read(MPU6050_RA_ACCEL_XOUT_H, buf, 6));
    for (int i = 0; i < 3; i++)
    {
        int16_t raw = (int16_t)(buf[2*i] << 8 | buf[2*i+1]); // combine registers
        accel[i] = (double)(raw - mpu6050_nonideal.accel_offset[i]) / mpu6050_nonideal.accel_gain[i]; // adjust to calibration
        accel[i] = accel[i] / 32768 * fsr; // scale to fsr
    }
}

/**
 * @brief Read 3-axis angular change of MPU6050 sensor registers and correct raw values
 */
static void mpu6050_get_gyro(double *gyro, int16_t fsr)
{
    uint8_t buf[6];
    ESP_ERROR_CHECK(mpu6050_read(MPU6050_RA_GYRO_XOUT_H, buf, 6));
    for (int i = 0; i < 3; i++)
    {
        int16_t raw = (int16_t)(buf[2*i] << 8 | buf[2*i+1]); // combine registers
        gyro[i] = (double)(raw - mpu6050_nonideal.gyro_offset[i]) / mpu6050_nonideal.gyro_gain[i]; // adjust to calibration
        gyro[i] = gyro[i] / 32768 * fsr;
    }
}

/**
 * @brief Calculate angles from 3-axis accelerometer values
 */
static void calc_angle_accel(double *curr_angle, double *accel, uint8_t degree_opt)
{
    double conversion = degree_opt ? (180. / M_PI) : 1; // degrees or radiant
    curr_angle[0] = atan(accel[1] / sqrt(pow(accel[0],2) + pow(accel[2],2))) * conversion; // roll
    curr_angle[1] = -atan(accel[0] / sqrt(pow(accel[1],2) + pow(accel[2],2))) * conversion; // pitch
}

/**
 * @brief Calculate angles from 3-axis gyroscope values
 */
static void calc_angle_gyro(double *curr_angle, double *gyro, int16_t t_sample, uint8_t degree_opt)
{
    double conversion = degree_opt ? (180. / M_PI) : 1; // degrees or radiant
    double scale = (float)(t_sample) / 1000.;
    for (int i = 0; i < 3; i++)
    {
        curr_angle[i] = curr_angle[i] + (gyro[i] * scale * conversion);
    }
}

/**
 * @brief Initialization of MPU6050
 */
static void mpu6050_init(int16_t fsr_accel, int16_t fsr_gyro)
{
    // wake mpu6050 up from sleep
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_RA_PWR_MGMT_1, 0 << MPU6050_PWR1_SLEEP_BIT));
    ESP_LOGI(TAG, "MPU6050: awake");

    // configure accerelometer
    int16_t MPU6050_ACCEL_FS;
    switch(fsr_accel){
        case 2: MPU6050_ACCEL_FS = MPU6050_ACCEL_FS_2; break;
        case 4: MPU6050_ACCEL_FS = MPU6050_ACCEL_FS_4; break;
        case 8: MPU6050_ACCEL_FS = MPU6050_ACCEL_FS_8; break;
        case 16: MPU6050_ACCEL_FS = MPU6050_ACCEL_FS_16; break;
        default: MPU6050_ACCEL_FS = MPU6050_ACCEL_FS_2;
    }
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS << MPU6050_ACONFIG_AFS_SEL_BIT));
    ESP_LOGI(TAG, "MPU6050: accel FSR --> +/- %dg", fsr_accel);

    // configure accerelometer
    int16_t MPU6050_GYRO_FS;
    switch(fsr_accel){
        case 250: MPU6050_GYRO_FS = MPU6050_GYRO_FS_250; break;
        case 500: MPU6050_GYRO_FS = MPU6050_GYRO_FS_500; break;
        case 1000: MPU6050_GYRO_FS = MPU6050_GYRO_FS_1000; break;
        case 2000: MPU6050_GYRO_FS = MPU6050_GYRO_FS_2000; break;
        default: MPU6050_GYRO_FS = MPU6050_GYRO_FS_250;
    }
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_RA_ACCEL_CONFIG, MPU6050_GYRO_FS << MPU6050_ACONFIG_AFS_SEL_BIT));
    ESP_LOGI(TAG, "MPU6050: gyro FSR --> +/- %d°/s", fsr_gyro);
}

void mpu6050_task(){

    // full scale ranges
    int16_t fsr_accel = 2;
    int16_t fsr_gyro = 250;

    // sampling
    int16_t t_sample = 50;
    double f_sample = (double)(1000 / t_sample);

    // sensor and task variables
    double *accel = (double *)calloc(3, sizeof(double));
    double *gyro = (double *)calloc(3, sizeof(double));
    double *angle_accel = (double *)calloc(2, sizeof(double)); // roll, pitch
    double *angle_gyro = (double *)calloc(3, sizeof(double)); // roll, pitch, yaw
    uint8_t degree_opt = 1;

    // configure mpu6050
    mpu6050_init(fsr_accel, fsr_gyro);

    while(1){

        // read registers
        mpu6050_get_accel(accel, fsr_accel);
        mpu6050_get_gyro(gyro, fsr_gyro);

        // basic angle calculation
        calc_angle_accel(angle_accel, accel, degree_opt);
        calc_angle_gyro(angle_gyro, gyro, t_sample, degree_opt);

        // 2D serial plotting
        printf("%2.2f,%2.2f,%2.2f,%2.2f\n", angle_accel[0], angle_accel[1], (float)((int16_t)(angle_gyro[0]) % 90), (float)((int16_t)(angle_gyro[1]) % 90));

        fflush(stdout);

        vTaskDelay(t_sample / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    i2c_config();
    xTaskCreate(&mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
}