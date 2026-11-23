#include <stdint.h>
#include <math.h> // powf() for altitude
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lps331ap.h"

// i2c
#define LPS331_ADDR 0x5D // 7-bit i2c address, SA0 = 1 on Pololu board; SA0 = 0 gives 0x5C = AM2320 address! 
#define LPS331_SCL_SPEED_HZ 100000 // 100kHz -> slow mode, 400kHz -> fast mode

// registers
#define LPS331_REG_WHO_AM_I 0x0F // device id register
#define LPS331_REG_RES_CONF 0x10 // resolution / averaging
#define LPS331_REG_CTRL_REG1 0x20 // power, data rate, BDU
#define LPS331_REG_CTRL_REG2 0x21 // boot, reset, one-shot
#define LPS331_REG_STATUS 0x27 // status register, first of the data block
#define LPS331_AUTO_INCREMENT 0x80 // MSB of register address -> auto increment

// register values
#define LPS331_WHO_AM_I_VALUE 0xBB // expected WHO_AM_I answer
#define LPS331_RES_CONF_VALUE 0x7A // max averaging: 512 pressure, 128 temp
#define LPS331_CTRL1_VALUE 0x84 // PD = 1 (active), ODR = 000 (one-shot), BDU = 1
#define LPS331_CTRL2_ONE_SHOT 0x01 // start one measurement

// status byte
#define LPS331_STATUS_P_DA 0x02 // bit[1]: new pressure data available

// data layout
#define LPS331_DATA_LEN 4 // status + 3 pressure bytes

// conversion
#define LPS331_PRESS_SCALE 4096.0f // LSB per mbar

// altitude, international barometric formula
#define LPS331_SEA_LEVEL_MBAR 1013.25f // standard sea level pressure
#define LPS331_ALT_EXPONENT 0.190263f // 1 / 5.255
#define LPS331_ALT_SCALE_M 44330.8f // meters

// delays in ms
#define LPS331_DELAY_POWER_ON_MS 100 // wait after power-on
#define LPS331_DELAY_MEASURE_MS 100 // one-shot time with max averaging


static const char *TAG = "lps331ap";


esp_err_t lps331_init (i2c_master_dev_handle_t *dev) {
	esp_err_t err;

	// (1) wait for sensor to wake up
	vTaskDelay(pdMS_TO_TICKS(LPS331_DELAY_POWER_ON_MS));

	// (2) adding device; the bus is already created in app_main 
	err = shared_i2c_add_device(LPS331_ADDR, LPS331_SCL_SPEED_HZ, dev);
	if (err != ESP_OK) {
		return err;
	}

	// (3) check WHO_AM_I: write register address, then read 1 byte
	uint8_t who_am_i_reg = LPS331_REG_WHO_AM_I;
	uint8_t who_am_i = 0;

	err = shared_i2c_write_read(*dev, &who_am_i_reg, 1, &who_am_i, 1);
	if (err != ESP_OK) {
		return err;
	}
	if (who_am_i == LPS331_WHO_AM_I_VALUE) {
		ESP_LOGI(TAG, "who_am_i: 0x%02X (expected 0x%02X)", who_am_i, LPS331_WHO_AM_I_VALUE);
	} else {
		ESP_LOGE(TAG, "who_am_i: 0x%02X (expected 0x%02X)", who_am_i, LPS331_WHO_AM_I_VALUE);
		return ESP_ERR_NOT_FOUND;
	}

	// (4) set resolution: register address + value
	uint8_t res_conf[2] = {LPS331_REG_RES_CONF, LPS331_RES_CONF_VALUE};
	err = shared_i2c_write(*dev, res_conf, sizeof(res_conf));
	if (err != ESP_OK) {
		return err;
	}

	// (5) power on in one-shot mode
	uint8_t ctrl1[2] = {LPS331_REG_CTRL_REG1, LPS331_CTRL1_VALUE};
	err = shared_i2c_write(*dev, ctrl1, sizeof(ctrl1));
	if (err != ESP_OK) {
		return err;
	}

	return ESP_OK;
}


esp_err_t lps331_read (i2c_master_dev_handle_t dev, float *press, float *alt) {
	uint8_t trigger_reading[2] = {LPS331_REG_CTRL_REG2, LPS331_CTRL2_ONE_SHOT};
	uint8_t data_reg = LPS331_REG_STATUS | LPS331_AUTO_INCREMENT;
	uint8_t data[LPS331_DATA_LEN];
	esp_err_t err;

	// (1) trigger measurement
	err = shared_i2c_write(dev, trigger_reading, sizeof(trigger_reading));
	if (err != ESP_OK) {
		return err;
	}

	// (2) wait until the sensor finishes measuring; the bus is free for other tasks meanwhile
	vTaskDelay(pdMS_TO_TICKS(LPS331_DELAY_MEASURE_MS));

	// (3) read 4 bytes starting at STATUS (0x27..0x2A) with auto increment
	err = shared_i2c_write_read(dev, &data_reg, 1, data, LPS331_DATA_LEN);
	if (err != ESP_OK) {
		return err;
	}

	// (4) check if new pressure data is ready
	if (!(data[0] & LPS331_STATUS_P_DA)) {
		return ESP_ERR_INVALID_STATE;
	}

	// (5) combine bits
	// press = 24 bits, two's complement -> data[1] (XL), data[2] (L), data[3] (H)
	uint32_t raw = ((uint32_t)data[3] << 16) | ((uint32_t)data[2] << 8) | data[1];
	if (raw & 0x800000) { // bit23: 0 - positive value, 1 - negative
		raw |= 0xFF000000;
	}
	int32_t raw_press = (int32_t)raw;


	// (6) convert to real values
	*press = raw_press / LPS331_PRESS_SCALE;
	*alt = (1.0f - powf(*press / LPS331_SEA_LEVEL_MBAR, LPS331_ALT_EXPONENT)) * LPS331_ALT_SCALE_M;

	return ESP_OK;
}








