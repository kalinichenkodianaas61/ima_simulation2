#include <stdint.h>
#include <math.h> // ldexpf(), powf()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bmp388.h"

// i2c
#define BMP388_ADDR 0x77 // 7-bit i2c address: SDO -> VDDIO = 0x77, SDO -> GND = 0x76
#define BMP388_SCL_SPEED_HZ 100000 // 100kHz -> standard mode, 400kHz -> fast mode

// registers
#define BMP388_REG_CHIP_ID 0x00 // chip identification
#define BMP388_REG_STATUS 0x03 // status flags, data registers 0x04..0x09 follow right after it
#define BMP388_REG_PWR_CTRL 0x1B // enable sensors + select power mode
#define BMP388_REG_OSR 0x1C // oversampling settings
#define BMP388_REG_CALIB 0x31 // first calibration register, NVM_PAR_T1 LSB
#define BMP388_REG_CMD 0x7E // command register

// commands and fixed values
#define BMP388_CHIP_ID 0x50 // expected value of CHIP_ID register
#define BMP388_CMD_SOFT_RESET 0xB6 // soft reset command

// PWR_CTRL bits
#define BMP388_PWR_PRESS_EN 0x01 // bit[0]: 1 - pressure sensor on
#define BMP388_PWR_TEMP_EN 0x02 // bit[1]: 1 - temperature sensor on
#define BMP388_PWR_MODE_FORCED 0x10 // bits[5:4] = 01 -> forced mode, one measurement then sleep

// OSR settings
#define BMP388_OSR_P_X8 0x03 // bits[2:0] = 011 -> pressure x8
#define BMP388_OSR_T_X1 0x00 // bits[5:3] = 000 -> temperature x1
#define BMP388_OSR_T_SHIFT 3 // osr_t starts at bit 3

// status byte
#define BMP388_STATUS_DRDY_PRESS 0x20 // bit[5]: 1 - new pressure data ready
#define BMP388_STATUS_DRDY_TEMP 0x40 // bit[6]: 1 - new temperature data ready
#define BMP388_STATUS_DRDY_MASK (BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP)

// data layout
#define BMP388_DATA_LEN 7 // status + 3 pressure bytes + 3 temperature bytes
#define BMP388_CALIB_LEN 21 // calibration registers 0x31..0x45

// conversion
#define BMP388_PA_IN_HPA 100.0f // 1 hPa = 100 Pa

// altitude, international barometric formula
// alt = 44330 * (1 - (p / p0)^(1 / 5.255))
#define BMP388_SEA_LEVEL_HPA 1013.25f // p0: standard sea-level pressure, put local QNH here for real altitude
#define BMP388_ALT_COEF 44330.0f // in meters
#define BMP388_ALT_EXP (1.0f / 5.255f) // exponent of the formula

// delays in ms
#define BMP388_DELAY_POWER_ON_MS 50 // wait after power-on, datasheet needs >= 2
#define BMP388_DELAY_RESET_MS 10 // wait after soft reset
#define BMP388_DELAY_MEASURE_MS 50 // measurement time, ~19 ms typ. for osr_p x8 + osr_t x1


static const char *TAG = "bmp388";




// read len bytes starting from register reg
// (static: helper is used only inside this file) 
static esp_err_t bmp388_read_regs (i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len) {
	// write register address, then repeated start + read
	return shared_i2c_write_read(dev, &reg, 1, buf, len);
}


// write one byte to register reg
static esp_err_t bmp388_write_reg (i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value) {
	uint8_t buf[2] = {reg, value};
	return shared_i2c_write(dev, buf, sizeof(buf));
}




esp_err_t bmp388_init (i2c_master_dev_handle_t *dev, bmp388_calib_t *calib) {
	esp_err_t err;

	// (1) wait for sensor to wake up
	vTaskDelay(pdMS_TO_TICKS(BMP388_DELAY_POWER_ON_MS));

	// (2) adding device; the bus is already created in app_main 
	err = shared_i2c_add_device(BMP388_ADDR, BMP388_SCL_SPEED_HZ, dev);
	if (err != ESP_OK) {
		return err;
	}

	// (3) check chip id
	uint8_t chip_id = 0;

	err = bmp388_read_regs(*dev, BMP388_REG_CHIP_ID, &chip_id, 1);
	if (err != ESP_OK) {
		return err;
	}
	if (chip_id == BMP388_CHIP_ID) {
		ESP_LOGI(TAG, "chip id: 0x%02X (expected 0x%02X)", chip_id, BMP388_CHIP_ID);
	} else {
		ESP_LOGE(TAG, "chip id: 0x%02X (expected 0x%02X)", chip_id, BMP388_CHIP_ID);
		return ESP_ERR_NOT_FOUND;
	}

	// (4) soft reset -> all settings to default
	err = bmp388_write_reg(*dev, BMP388_REG_CMD, BMP388_CMD_SOFT_RESET);
	if (err != ESP_OK) {
		return err;
	}
	vTaskDelay(pdMS_TO_TICKS(BMP388_DELAY_RESET_MS));

	// (5) read calibration coefficients 0x31..0x45 in one burst
	uint8_t c[BMP388_CALIB_LEN];

	err = bmp388_read_regs(*dev, BMP388_REG_CALIB, c, BMP388_CALIB_LEN);
	if (err != ESP_OK) {
		return err;
	}

	// (6) combine bytes into raw NVM values, LSB first
	// types from datasheet table 22
	uint16_t nvm_t1 = (uint16_t)((c[1] << 8) | c[0]);
	uint16_t nvm_t2 = (uint16_t)((c[3] << 8) | c[2]);
	int8_t nvm_t3 = (int8_t)c[4];
	int16_t nvm_p1 = (int16_t)((c[6] << 8) | c[5]);
	int16_t nvm_p2 = (int16_t)((c[8] << 8) | c[7]);
	int8_t nvm_p3 = (int8_t)c[9];
	int8_t nvm_p4 = (int8_t)c[10];
	uint16_t nvm_p5 = (uint16_t)((c[12] << 8) | c[11]);
	uint16_t nvm_p6 = (uint16_t)((c[14] << 8) | c[13]);
	int8_t nvm_p7 = (int8_t)c[15];
	int8_t nvm_p8 = (int8_t)c[16];
	int16_t nvm_p9 = (int16_t)((c[18] << 8) | c[17]);
	int8_t nvm_p10 = (int8_t)c[19];
	int8_t nvm_p11 = (int8_t)c[20];

	// (7) convert to float, datasheet section 9.1
	// ldexpf(x, n) = x * 2^n, negative n -> division by 2^|n|
	// calib->par_t1 = ...; is equivalent to (*calib).par_t1 = ...;
	calib->par_t1 = ldexpf((float)nvm_t1, 8); // / 2^-8
	calib->par_t2 = ldexpf((float)nvm_t2, -30); // / 2^30
	calib->par_t3 = ldexpf((float)nvm_t3, -48); // / 2^48
	calib->par_p1 = ldexpf((float)nvm_p1 - 16384.0f, -20); // (P1 - 2^14) / 2^20
	calib->par_p2 = ldexpf((float)nvm_p2 - 16384.0f, -29); // (P2 - 2^14) / 2^29
	calib->par_p3 = ldexpf((float)nvm_p3, -32); // / 2^32
	calib->par_p4 = ldexpf((float)nvm_p4, -37); // / 2^37
	calib->par_p5 = ldexpf((float)nvm_p5, 3); // / 2^-3
	calib->par_p6 = ldexpf((float)nvm_p6, -6); // / 2^6
	calib->par_p7 = ldexpf((float)nvm_p7, -8); // / 2^8
	calib->par_p8 = ldexpf((float)nvm_p8, -15); // / 2^15
	calib->par_p9 = ldexpf((float)nvm_p9, -48); // / 2^48
	calib->par_p10 = ldexpf((float)nvm_p10, -48); // / 2^48
	calib->par_p11 = ldexpf((float)nvm_p11, -65); // / 2^65
	calib->t_lin = 0.0f;

	// (8) set oversampling: pressure x8, temperature x1
	err = bmp388_write_reg(*dev, BMP388_REG_OSR, (BMP388_OSR_T_X1 << BMP388_OSR_T_SHIFT) | BMP388_OSR_P_X8);
	if (err != ESP_OK) {
		return err;
	}

	return ESP_OK;
}




// temperature compensation, datasheet section 9.2, result in Celsius
static float bmp388_compensate_temp (uint32_t raw_temp, bmp388_calib_t *calib) {
	float d1 = (float)raw_temp - calib->par_t1;
	float d2 = d1 * calib->par_t2;

	// t_lin is saved because pressure formula needs it
	calib->t_lin = d2 + (d1 * d1) * calib->par_t3;
	return calib->t_lin;
}


// pressure compensation, datasheet section 9.3, result in Pa
// call only after bmp388_compensate_temp (it updates t_lin)
static float bmp388_compensate_press (uint32_t raw_press, const bmp388_calib_t *calib) {
	float t = calib->t_lin;
	float t2 = t * t;
	float t3 = t2 * t;
	float p = (float)raw_press;

	// offset part
	float out1 = calib->par_p5 + calib->par_p6 * t + calib->par_p7 * t2 + calib->par_p8 * t3;

	// sensitivity part
	float out2 = p * (calib->par_p1 + calib->par_p2 * t + calib->par_p3 * t2 + calib->par_p4 * t3);

	// higher-order part
	float out3 = p * p * (calib->par_p9 + calib->par_p10 * t) + p * p * p * calib->par_p11;

	return out1 + out2 + out3;
}




// altitude from pressure, result in meters
static float bmp388_altitude (float press_hpa) {
	return BMP388_ALT_COEF * (1.0f - powf(press_hpa / BMP388_SEA_LEVEL_HPA, BMP388_ALT_EXP));
}




esp_err_t bmp388_read (i2c_master_dev_handle_t dev, bmp388_calib_t *calib, float *press, float *alt) {
	uint8_t data[BMP388_DATA_LEN];
	esp_err_t err;

	// (1) trigger one measurement in forced mode
	err = bmp388_write_reg(dev, BMP388_REG_PWR_CTRL, BMP388_PWR_PRESS_EN | BMP388_PWR_TEMP_EN | BMP388_PWR_MODE_FORCED);
	if (err != ESP_OK) {
		return err;
	}

	// (2) wait until the sensor finishes measuring; the bus is free for other tasks meanwhile
	vTaskDelay(pdMS_TO_TICKS(BMP388_DELAY_MEASURE_MS));

	// (3) read 7 bytes in one burst: status + 3 pressure + 3 temperature
	err = bmp388_read_regs(dev, BMP388_REG_STATUS, data, BMP388_DATA_LEN);
	if (err != ESP_OK) {
		return err;
	}

	// (4) check if both data are ready
	if ((data[0] & BMP388_STATUS_DRDY_MASK) != BMP388_STATUS_DRDY_MASK) {
		return ESP_ERR_INVALID_STATE;
	}

	// (5) combine bits, 24-bit unsigned, LSB first (xlsb, lsb, msb)
	// press -> data[1] xlsb, data[2] lsb, data[3] msb
	// temp  -> data[4] xlsb, data[5] lsb, data[6] msb
	uint32_t raw_press = ((uint32_t)data[3] << 16) | ((uint32_t)data[2] << 8) | ((uint32_t)data[1]);
	uint32_t raw_temp = ((uint32_t)data[6] << 16) | ((uint32_t)data[5] << 8) | ((uint32_t)data[4]);

	// (6) temperature is not returned, but must be computed first: it updates t_lin for pressure
	bmp388_compensate_temp(raw_temp, calib);

	// (7) convert to real values
	*press = bmp388_compensate_press(raw_press, calib) / BMP388_PA_IN_HPA;
	*alt = bmp388_altitude(*press);

	return ESP_OK;
}







