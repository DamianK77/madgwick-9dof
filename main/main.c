#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_mpu6050.h"
#include "esp_hmc5883l.h"

#define I2C_NUM        I2C_NUM_0
#define I2C_SDA_PIN    GPIO_NUM_3
#define I2C_SCL_PIN    GPIO_NUM_9
#define I2C_FREQ       400000

i2c_master_bus_handle_t bus_handle;

static const char* TAG = "main";

void madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    // Madgwick algorithm
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

    // probe sensors
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, mpu6050_conf.i2c_addr, MPU6050_TIMEOUT_MS));
    ESP_ERROR_CHECK(i2c_master_probe(bus_handle, HMC5883L_ADDR, HMC5883L_TIMEOUT_MS));

    ESP_LOGI(TAG, "Calibrating MPU6050 in 5 seconds, keep the sensor still...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    // measure and calculate offsets
    float x_gyro_offset = 0;
    float y_gyro_offset = 0;
    float z_gyro_offset = 0;
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
        float x_gyro, y_gyro, z_gyro;
        mpu6050_read_gyroscope(mpu6050_conf, &x_gyro, &y_gyro, &z_gyro); // in degrees per second
        // adjust for offsets
        x_gyro -= x_gyro_offset;
        y_gyro -= y_gyro_offset;
        z_gyro -= z_gyro_offset;
        ESP_LOGI(TAG, "Gyroscope: x=%.2f, y=%.2f, z=%.2f", x_gyro, y_gyro, z_gyro);

        float x_acc, y_acc, z_acc;
        mpu6050_read_accelerometer(mpu6050_conf, &x_acc, &y_acc, &z_acc); // in g
        ESP_LOGI(TAG, "Accelerometer: x=%.2f, y=%.2f, z=%.2f", x_acc, y_acc, z_acc);

        float x_mag, y_mag, z_mag;
        hmc5883l_read_magnetometer(hmc5883l_conf, &x_mag, &y_mag, &z_mag); // in gauss
        ESP_LOGI(TAG, "Magnetometer: x=%.2f, y=%.2f, z=%.2f", x_mag, y_mag, z_mag);

        // wait
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    

}