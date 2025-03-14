#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mpu6050.h"
#include "esp_hmc5883l.h"

#define I2C_NUM        I2C_NUM_0
#define I2C_SDA_PIN    GPIO_NUM_3
#define I2C_SCL_PIN    GPIO_NUM_9
#define I2C_FREQ       400000

// values for low pass filter
#define BETA_A         0.1
#define BETA_G         0.1
#define BETA_M         1.0

i2c_master_bus_handle_t bus_handle;
float x_gyro_offset = 0;
float y_gyro_offset = 0;
float z_gyro_offset = 0;

float ax_prev = 0, ay_prev = 0, az_prev = 0; // used by low pass filter
float gx_prev = 0, gy_prev = 0, gz_prev = 0;
float mx_prev = 0, my_prev = 0, mz_prev = 0;

static const char* TAG = "main";

// structure type definition for imu config containing mpu6050 and hmc5883l configurations
typedef struct {
    mpu6050_conf_t mpu6050_conf;
    hmc5883l_conf_t hmc5883l_conf;
} imu_conf_t;

void madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    // Madgwick algorithm
}

void read_imu(imu_conf_t imu_conf, float *gx, float *gy, float *gz, float *ax, float *ay, float *az, float *mx, float *my, float *mz) {
    float x_gyro, y_gyro, z_gyro;
    mpu6050_read_gyroscope(imu_conf.mpu6050_conf, &x_gyro, &y_gyro, &z_gyro); // in degrees per second
    // adjust for offsets
    x_gyro -= x_gyro_offset;
    y_gyro -= y_gyro_offset;
    z_gyro -= z_gyro_offset;

    float x_acc, y_acc, z_acc;
    mpu6050_read_accelerometer(imu_conf.mpu6050_conf, &x_acc, &y_acc, &z_acc); // in g

    float x_mag, y_mag, z_mag;
    hmc5883l_read_magnetometer(imu_conf.hmc5883l_conf, &x_mag, &y_mag, &z_mag); // in gauss

    // low pass filter
    x_acc = BETA_A * x_acc + (1 - BETA_A) * ax_prev;
    y_acc = BETA_A * y_acc + (1 - BETA_A) * ay_prev;
    z_acc = BETA_A * z_acc + (1 - BETA_A) * az_prev;

    x_gyro = BETA_G * x_gyro + (1 - BETA_G) * gx_prev;
    y_gyro = BETA_G * y_gyro + (1 - BETA_G) * gy_prev;
    z_gyro = BETA_G * z_gyro + (1 - BETA_G) * gz_prev;

    x_mag = BETA_M * x_mag + (1 - BETA_M) * mx_prev;
    y_mag = BETA_M * y_mag + (1 - BETA_M) * my_prev;
    z_mag = BETA_M * z_mag + (1 - BETA_M) * mz_prev;

    ax_prev = x_acc;
    ay_prev = y_acc;
    az_prev = z_acc;

    gx_prev = x_gyro;
    gy_prev = y_gyro;
    gz_prev = z_gyro;

    mx_prev = x_mag;
    my_prev = y_mag;
    mz_prev = z_mag;

    *gx = x_gyro;
    *gy = y_gyro;
    *gz = z_gyro;
    *ax = x_acc;
    *ay = y_acc;
    *az = z_acc;
    *mx = x_mag;
    *my = y_mag;
    *mz = z_mag;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C master");
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM,
        .scl_io_num = I2C_SCL_PIN,
        .sda_io_num = I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    i2c_new_master_bus(&i2c_mst_config, &bus_handle);

    ESP_LOGI(TAG, "Starting MPU6050");
    mpu6050_conf_t mpu6050_conf = {
        .sda_pin = I2C_SDA_PIN,
        .scl_pin = I2C_SCL_PIN,
        .i2c_freq = I2C_FREQ,
        .i2c_addr = MPU6050_ADDR_0,
        .i2c_port = I2C_NUM
    };
    mpu6050_init(mpu6050_conf); 

    // set the mpu to passthrough mode
    mpu6050_i2c_passthrough(mpu6050_conf);

    ESP_LOGI(TAG, "MPU6050 initialized");

    // initialize HMC5883L
    hmc5883l_conf_t hmc5883l_conf = {
        .sda_pin = I2C_SDA_PIN,
        .scl_pin = I2C_SCL_PIN,
        .i2c_freq = I2C_FREQ,
        .i2c_port = I2C_NUM
    };
    hmc5883l_init(hmc5883l_conf);
    hmc5883l_write_config(hmc5883l_conf, HMC5883L_OVER_SAMPLE_8, HMC5883L_DATA_OUTPUT_RATE_75_HZ, HMC5883L_MODE_NORMAL, HMC5883L_GAIN_1090);
    hmc5883l_write_mode(hmc5883l_conf, HMC5883L_CONTINUOUS_MODE);

    imu_conf_t imu_conf = {
        .mpu6050_conf = mpu6050_conf,
        .hmc5883l_conf = hmc5883l_conf
    };

    // probe sensors
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, mpu6050_conf.i2c_addr, MPU6050_TIMEOUT_MS));
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, HMC5883L_ADDR, HMC5883L_TIMEOUT_MS));

    ESP_LOGI(TAG, "Calibrating MPU6050 in 5 seconds, keep the sensor still...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    // measure and calculate offsets
    for (int i = 0; i < 100; i++)
    {
        float x_gyro, y_gyro, z_gyro;
        mpu6050_read_gyroscope(mpu6050_conf, &x_gyro, &y_gyro, &z_gyro);
        x_gyro_offset += x_gyro;
        y_gyro_offset += y_gyro;
        z_gyro_offset += z_gyro;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    x_gyro_offset /= 100;
    y_gyro_offset /= 100;
    z_gyro_offset /= 100;
    ESP_LOGI(TAG, "Gyroscope offsets: x=%.2f, y=%.2f, z=%.2f", x_gyro_offset, y_gyro_offset, z_gyro_offset);


    // ESP_LOGI(TAG, "Calibrating HMC5883L in 5 seconds...");
    // vTaskDelay(5000 / portTICK_PERIOD_MS);
    // hmc5883l_calibrate(hmc5883l_conf);

    while (true)
    {
        // read data
        float gx, gy, gz, ax, ay, az, mx, my, mz;

        // measure time taken to read data
        // uint64_t start_time = esp_timer_get_time();
        read_imu(imu_conf, &gx, &gy, &gz, &ax, &ay, &az, &mx, &my, &mz);
        // uint64_t end_time = esp_timer_get_time();
        // ESP_LOGI(TAG, "Time taken to read data: %llu us", end_time - start_time);

        // print data for teleplot
        printf(">gx:%.2f\n>gy:%.2f\n>gz:%.2f\n>ax:%.2f\n>ay:%.2f\n>az:%.2f\n>mx:%.2f\n>my:%.2f\n>mz:%.2f\n", gx, gy, gz, ax, ay, az, mx, my, mz);
        // wait
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    

}