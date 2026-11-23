#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mpu6050.h"

// i2c
#define MPU6050_ADDR 0x68 // AD0 -> GND
#define MPU6050_SCL_SPEED_HZ 100000 // 100kHz -> slow mode, 400kHz -> fast mode

// registers
#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_WHO_AM_I_VAL 0x68 // expected WHO_AM_I value 
#define MPU6050_ACCEL_CFG_REG 0x1C
#define MPU6050_AFS_SEL 0x00 // 0 -> +-2g
#define MPU6050_PWR_MGMT1_REG 0x6B
#define MPU6050_PWR_MGMT1_ARG 0x01 // enable PLL with X axis gyroscope reference
#define MPU6050_START_REG 0x3B

// data layout 
#define MPU6050_DATA_LEN 6 // 3 axes * 2 bytes 

// conversion
#define MPU6050_LSB_SENSITIVITY 16384.0f

// delays
#define MPU6050_DELAY_WAKE_UP_MS 100 // wait after sensor wakes up


static const char *TAG = "mpu6050";


esp_err_t mpu6050_init (i2c_master_dev_handle_t *dev) {
	esp_err_t err;

	// (1) adding device; the bus is already created in app_main 
	err = shared_i2c_add_device(MPU6050_ADDR, MPU6050_SCL_SPEED_HZ, dev);
	if (err != ESP_OK) {
		return err;
	}


	// (2) checking WHO_AM_I register
	uint8_t who_am_i_reg = MPU6050_WHO_AM_I_REG;
	uint8_t who = 0;

	err = shared_i2c_write_read(*dev, &who_am_i_reg, 1, &who, 1);
	if (err != ESP_OK) {
		return err;
	}

	if (who == MPU6050_WHO_AM_I_VAL) {
		ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);
	} else {
		ESP_LOGW(TAG, "unexpected WHO_AM_I: 0x%02X", who);
	}

	// (3) waking up the mpu and set gyro clk source
	uint8_t wake[2] = {MPU6050_PWR_MGMT1_REG, MPU6050_PWR_MGMT1_ARG};

	err = shared_i2c_write(*dev, wake, sizeof(wake));
	if (err != ESP_OK) {
		return err;
	}
	vTaskDelay(pdMS_TO_TICKS(MPU6050_DELAY_WAKE_UP_MS)); // wait for sensor startup


	// (4) set accel range to +-2g explicitly
	uint8_t acc_cfg[2] = {MPU6050_ACCEL_CFG_REG, MPU6050_AFS_SEL};
	err = shared_i2c_write(*dev, acc_cfg, sizeof(acc_cfg));
	if (err != ESP_OK) {
		return err;
	}

	return ESP_OK;
}


esp_err_t mpu6050_read (i2c_master_dev_handle_t dev, float *gX, float *gY, float *gZ) {
	uint8_t start_reg = MPU6050_START_REG;
	uint8_t data[MPU6050_DATA_LEN];
	esp_err_t err;

	// (1) read values
	err = shared_i2c_write_read(dev, &start_reg, 1, data, MPU6050_DATA_LEN);
	if (err != ESP_OK) {
		return err;
	}

	// (2) combine bits
	int16_t rawX = (int16_t) ((data[0] << 8) | data[1]);
	int16_t rawY = (int16_t) ((data[2] << 8) | data[3]);
	int16_t rawZ = (int16_t) ((data[4] << 8) | data[5]);

	// (3) convert to real values
	*gX = rawX / MPU6050_LSB_SENSITIVITY;
	*gY = rawY / MPU6050_LSB_SENSITIVITY;
	*gZ = rawZ / MPU6050_LSB_SENSITIVITY;

	return ESP_OK;
}









