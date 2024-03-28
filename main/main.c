#include "driver/i2c.h"
#include "sys/time.h"



#define I2C_MASTER_SCL_IO    9    // Enter your SCL pin here

#define I2C_MASTER_SDA_IO    8    // Enter your SDA pin here

#define I2C_MASTER_NUM       I2C_NUM_0

#define I2C_MASTER_FREQ_HZ   2000

#define I2C_MASTER_TX_BUF_DISABLE 0

#define I2C_MASTER_RX_BUF_DISABLE 0



// Initialize the I2C master driver

static void i2c_master_init(void) {

    i2c_config_t i2c_config = {

        .mode = I2C_MODE_MASTER,

        .sda_io_num = I2C_MASTER_SDA_IO,

        .scl_io_num = I2C_MASTER_SCL_IO,

        .sda_pullup_en = GPIO_PULLUP_ENABLE,

        .scl_pullup_en = GPIO_PULLUP_ENABLE,

        .master.clk_speed = I2C_MASTER_FREQ_HZ,

    };

    i2c_param_config(I2C_MASTER_NUM, &i2c_config);

    i2c_driver_install(I2C_MASTER_NUM, i2c_config.mode,

                       I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0);

}







#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "esp_system.h"

#include "esp_log.h"

#include "mpu6050.h"



#define ACCELEROMETER_TAG "ACCELEROMETER"

#define GYROSCOPE_TAG "GYROSCOPE"

#define SAMPLE_FREQUENCY_HZ 200 // Frequency in Hertz

#define SAMPLE_PERIOD (1000 / SAMPLE_FREQUENCY_HZ) // Period in milliseconds

#define SAMPLE_DURATION 3 // Duration in seconds



void app_main(void) {

    i2c_master_init();

    i2c_port_t i2c_port = I2C_NUM_0; 

    mpu6050_handle_t mpu6050 = mpu6050_create(i2c_port, MPU6050_I2C_ADDRESS);

    if (mpu6050 == NULL) {

        ESP_LOGE("mpu6050", "Could not create MPU6050 handle");

        return;

    }



    // Wake up the MPU6050

    if (mpu6050_wake_up(mpu6050) != ESP_OK) {

        ESP_LOGE("mpu6050", "Could not wake up MPU6050");

        return;

    }



    // Configure MPU6050 settings

    if (mpu6050_config(mpu6050, ACCE_FS_2G, GYRO_FS_250DPS) != ESP_OK) {

        ESP_LOGE("mpu6050", "Could not configure MPU6050");

        return;

    }

    struct timeval tv_now;


    while (1) {

        // Get the accelerometer values

        gettimeofday(&tv_now, NULL);
        long time_in_mill =
            (tv_now.tv_sec) * 1000 + (tv_now.tv_usec) / 1000; 

        mpu6050_acce_value_t acce_value;
        if (mpu6050_get_acce(mpu6050, &acce_value) == ESP_OK) {
            ESP_LOGI(ACCELEROMETER_TAG, "Time: [%ld ms] Accelerometer: x=%.2f, y=%.2f, z=%.2f",
                     time_in_mill, acce_value.acce_x, acce_value.acce_y, acce_value.acce_z);
        }



        // Get the gyroscope values

        mpu6050_gyro_value_t gyro_value;
        if (mpu6050_get_gyro(mpu6050, &gyro_value) == ESP_OK) {
            ESP_LOGI(GYROSCOPE_TAG, "Time: [%ld ms] Gyroscope: x=%.2f, y=%.2f, z=%.2f",
                     time_in_mill, gyro_value.gyro_x, gyro_value.gyro_y, gyro_value.gyro_z);
        }



        // Delay before the next read

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD));
    }

}