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
#define I2C_FREQ       1000000 // 1MHz

// values for low pass filter
#define BETA_A         0.5
#define BETA_G         0.5
#define BETA_M         1.0

i2c_master_bus_handle_t bus_handle;
float x_gyro_offset = 0;
float y_gyro_offset = 0;
float z_gyro_offset = 0;

float ax_prev = 0, ay_prev = 0, az_prev = 0; // used by low pass filter
float gx_prev = 0, gy_prev = 0, gz_prev = 0;
float mx_prev = 0, my_prev = 0, mz_prev = 0;

float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float roll_IMU = 0.0f;
float pitch_IMU = 0.0f;
float yaw_IMU = 0.0f;

const float beta_madgwick = 0.1f;  //Madgwick filter parameter

static const char* TAG = "main";

// structure type definition for imu config containing mpu6050 and hmc5883l configurations
typedef struct {
    mpu6050_conf_t mpu6050_conf;
    hmc5883l_conf_t hmc5883l_conf;
} imu_conf_t;

float invSqrt(float x) {
    float xhalf = 0.5f * x;
    union {
        float f;
        int i;
    } conv = { .f = x };
    conv.i = 0x5f3759df - (conv.i >> 1);
    x = conv.f;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

float constrain(float x, float min, float max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    } else {
        return x;
    }
}


void madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    // Madgwick algorithm, dt is the time difference between readings, needed for integration
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    //Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    //Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        //Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        //Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        //Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        //Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        //gradient = gradient/gradient_norm 
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        //Apply feedback step
        //quat change = quat change - self gain * gradient
        qDot1 -= beta_madgwick * s0;
        qDot2 -= beta_madgwick * s1;
        qDot3 -= beta_madgwick * s2;
        qDot4 -= beta_madgwick * s3;
    }

    //Integrate rate of change of quaternion to yield quaternion
    //estimation quaternion = estimation quaternion + quat change * dt
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    //Normalize quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    
    //compute angles - NWU
    roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
    pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
    yaw_IMU = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
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
    // enable fifo on all sensors
    // mpu6050_set_fifo_enable(mpu6050_conf, true, true, true, true, true, true);

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

    float start_time = esp_timer_get_time();

    while (true)
    {
        // read data
        float gx, gy, gz, ax, ay, az, mx, my, mz;

        // measure time taken for a single measurement
        uint64_t start_time_performance = esp_timer_get_time();

        // uint64_t start_time = esp_timer_get_time();
        read_imu(imu_conf, &gx, &gy, &gz, &ax, &ay, &az, &mx, &my, &mz);
        uint64_t end_time_performance = esp_timer_get_time();
        ESP_LOGI(TAG, "Time taken to read data: %llu us", end_time_performance - start_time_performance);
        // uint64_t end_time = esp_timer_get_time();
        // ESP_LOGI(TAG, "Time taken to read data: %llu us", end_time - start_time);

        // madgwick filter
        float dt = (esp_timer_get_time() - start_time) / 1000000.0f;
        start_time = esp_timer_get_time();
        madgwick(gx, gy, gz, ax, ay, az, mx, my, mz, dt);


        // print data
        printf(">Roll:%.2f\n>Pitch:%.2f\n>Yaw:%.2f\n", roll_IMU, pitch_IMU, yaw_IMU);

        printf(">3D|mySimpleCube:S:cube:P:1:1:1:R:%.2f:%.2f:%.2f:W:2:H:1:D:2:C:#2ecc71\n", roll_IMU * 0.0174532925f, pitch_IMU * 0.0174532925f, yaw_IMU * 0.0174532925f); // data should be in radian

        // print data for teleplot
        // printf(">gx:%.2f\n>gy:%.2f\n>gz:%.2f\n>ax:%.2f\n>ay:%.2f\n>az:%.2f\n>mx:%.2f\n>my:%.2f\n>mz:%.2f\n", gx, gy, gz, ax, ay, az, mx, my, mz);
        // wait
        // vTaskDelay(10 / portTICK_PERIOD_MS);

    }
    

}