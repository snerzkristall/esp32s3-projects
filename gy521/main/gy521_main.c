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
    .accel_offset = {520, 0, -1550}, // TODO recalibrate with servos
    .accel_gain = {0.9973, 1.0022, 1.0162}, // TODO recalibrate with servos
    .temp_offset = 0, // TODO calibrate
    .temp_gain = 1, // TODO calibrate
    .gyro_offset = {-241, -239, -221},
    .gyro_gain = {1, 1, 1} // TODO calibrate
};

typedef struct{
    float I;
    float A;
    float A_T;
    float B;
    float H;
    float H_T;
    float Q;
    float R;
} KALMAN_FILTER_PARAMS;

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
static void mpu6050_get_accel(float *accel, int16_t fsr)
{
    uint8_t buf[6];
    int16_t raw;
    ESP_ERROR_CHECK(mpu6050_read(MPU6050_RA_ACCEL_XOUT_H, buf, 6));
    for (int i = 0; i < 3; i++)
    {
        raw = (int16_t)(buf[2*i] << 8 | buf[2*i+1]); // combine registers
        accel[i] = (float)(raw - mpu6050_nonideal.accel_offset[i]) / mpu6050_nonideal.accel_gain[i]; // adjust to calibration
        accel[i] = accel[i] / 32768. * fsr; // scale to fsr
    }
}

/**
 * @brief Read 3-axis angular change of MPU6050 sensor registers and correct raw values
 */
static void mpu6050_get_gyro(float *gyro, int16_t fsr)
{
    uint8_t buf[6];
    int16_t raw;
    ESP_ERROR_CHECK(mpu6050_read(MPU6050_RA_GYRO_XOUT_H, buf, 6));
    for (int i = 0; i < 3; i++)
    {
        raw = (int16_t)(buf[2*i] << 8 | buf[2*i+1]); // combine registers
        gyro[i] = (float)(raw - mpu6050_nonideal.gyro_offset[i]) / mpu6050_nonideal.gyro_gain[i]; // adjust to calibration
        gyro[i] = gyro[i] / 32768. * fsr;
    }
}

/**
 * @brief Calculate angles from 3-axis accelerometer values
 */
static void calc_angle_accel(float *curr_angle, float *accel, uint8_t degree_opt)
{
    float conversion = degree_opt ? (180. / M_PI) : 1; // degrees or radiant
    curr_angle[0] = atanf(accel[1] / sqrtf(powf(accel[0],2) + powf(accel[2],2))) * conversion; // roll
    curr_angle[1] = -atanf(accel[0] / sqrtf(powf(accel[1],2) + powf(accel[2],2))) * conversion; // pitch
}

/**
 * @brief Calculate angles from 3-axis gyroscope values
 */
static void calc_angle_gyro(float *curr_angle, float *gyro, int16_t t_sample, uint8_t degree_opt)
{
    float conversion = degree_opt ? 1 : (M_PI / 180.); // degrees or radiant
    float t_step = (float)(t_sample) / 1000.;
    for (int i = 0; i < 3; i++)
    {
        curr_angle[i] += (gyro[i] * t_step * conversion);
    }
}

/**
 * @brief Kalman filter 1D (scalar)
 * 
 * State model with input
 * -----------------------------------
 * x_k = A * x_k-1 + B * u_k-1 + q_k-1
 * y_k = H * x_k + r_k
 * -----------------------------------
 * 
 * Buffer content
 * -----------------------------------
 * [  0  |  1  |   2   |   3   |   4   |  5  ]
 * [ x_k | M_k | x_k-1 | M_k-1 | u_k-1 | y_k ]
 */
static void kalman_filter(KALMAN_FILTER_PARAMS *kmf, float *buf)
{
    // prediction
    float x_k_pred = kmf->A * buf[2] + kmf->B * buf[4];
    float M_k_pred = kmf->A * buf[3] * kmf->A_T + kmf->Q;

    // update
    float K = M_k_pred * kmf->H * 1 / (kmf->H * M_k_pred * kmf->H_T + kmf->R);
    buf[0] = x_k_pred + K * (buf[5] - x_k_pred);
    buf[1] = (kmf->I - K * kmf->H) * M_k_pred;

    // store current sample
    buf[2] = buf[0];
    buf[3] = buf[1];
}

/**
 * @brief Initialization of MPU6050
 */
static void mpu6050_init()
{
    // wake mpu6050 up from sleep
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_RA_PWR_MGMT_1, 0 << MPU6050_PWR1_SLEEP_BIT));
    ESP_LOGI(TAG, "MPU6050: awake");

    // configure accerelometer
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACCEL_FS_2 << MPU6050_ACONFIG_AFS_SEL_BIT));
    ESP_LOGI(TAG, "MPU6050: accel FSR --> +/- 2g");

    // configure accerelometer
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_RA_ACCEL_CONFIG, MPU6050_GYRO_FS_250 << MPU6050_ACONFIG_AFS_SEL_BIT));
    ESP_LOGI(TAG, "MPU6050: gyro FSR --> +/- 250°/s");
}

void mpu6050_task(){

    // full scale ranges
    int16_t fsr_accel = 2;
    int16_t fsr_gyro = 250;

    // sampling
    int16_t f_sample = 20; // Hz
    int16_t t_sample = 1000 / f_sample; // milliseconds

    // sensor and task variables
    float *accel = (float *)calloc(3, sizeof(float));
    float *gyro = (float *)calloc(3, sizeof(float));
    float *angle_accel = (float *)calloc(2, sizeof(float)); // roll, pitch
    uint8_t degree_opt = 1;

    // kalman filter
    float *kalman_roll = (float *)calloc(6, sizeof(float));
    float *kalman_pitch = (float *)calloc(6, sizeof(float));
    kalman_roll[1] = (float)(2*2); // init TODO find starting value
    kalman_pitch[1] = (float)(2*2); // init TODO find starting value
    float sigma_gyro = 4; // TODO find sigma
    float sigma_angle = 3; // TODO find sigma
    KALMAN_FILTER_PARAMS kmf = {
        .I = 1,
        .A = 1,
        .A_T = 1,
        .B = (float)(1000. / t_sample),
        .H = 1,
        .H_T = 1,
        .Q = powf((float)(1000. / t_sample),2) * powf(sigma_gyro,2),
        .R = powf(sigma_angle,2)
    };

    // configure mpu6050
    mpu6050_init();

    while(1){

        // read registers
        mpu6050_get_accel(accel, fsr_accel);
        mpu6050_get_gyro(gyro, fsr_gyro);

        // basic angle calculation
        calc_angle_accel(angle_accel, accel, degree_opt);

        // kalman filter
        kalman_roll[4] = gyro[0]; // u_k-1
        kalman_roll[5] = angle_accel[0]; // y_k
        kalman_filter(&kmf, kalman_roll);
        kalman_pitch[4] = gyro[1]; // u_k-1
        kalman_pitch[5] = angle_accel[1]; // y_k
        kalman_filter(&kmf, kalman_pitch);

        // serial plotting
        printf("%3.1f,%3.1f,%3.1f,%3.1f\n", angle_accel[0], angle_accel[1], kalman_roll[0], kalman_pitch[0]);

        fflush(stdout);

        vTaskDelay(t_sample / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    i2c_config();
    xTaskCreate(&mpu6050_task, "mpu6050_task", 2048, NULL, 5, NULL);
}