#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "am2320.h"

// i2c
#define AM2320_ADDR 0x5C // 7-bit i2c address; datasheet gives 0xB8 = 0x5C << 1 with R/W bit
#define AM2320_SCL_SPEED_HZ 100000 // 100kHz, datasheet maximum for AM2320

// commands (I2C-Modbus)
#define AM2320_CMD_READ_REGS 0x03 // function code: read registers
#define AM2320_REG_HUM_HIGH 0x00 // first data register: humidity high byte
#define AM2320_DATA_REG_COUNT 4 // hum high/low + temp high/low
#define AM2320_REG_MODEL_HIGH 0x08 // first info register: model high byte
#define AM2320_INFO_REG_COUNT 7 // model (2) + version (1) + device ID (4)
#define AM2320_MAX_REGS 10 // sensor returns max 10 registers per read

// response layout
#define AM2320_HEADER_LEN 2 // function code + byte count
#define AM2320_CRC_LEN 2 // CRC16, low byte first

// crc16 (Modbus)
#define AM2320_CRC_INIT 0xFFFF // initial CRC register value
#define AM2320_CRC_POLY 0xA001 // reversed polynomial from datasheet

// conversion
#define AM2320_TEMP_SIGN_BIT 0x8000 // bit15 of temp: 1 - negative
#define AM2320_TEMP_VALUE_MASK 0x7FFF // bit14..bit0: temp value
#define AM2320_RAW_DIVIDER 10.0f // raw value = real value * 10

// delays in ms
#define AM2320_DELAY_POWER_ON_MS 2000 // wait after power-on, datasheet: 2 s, no commands allowed
#define AM2320_DELAY_WAKE_MS 10 // pause after wake-up, datasheet needs >= 0.8
#define AM2320_DELAY_CMD_MS 10 // pause between read command and reading data, datasheet needs >= 1.5
#define AM2320_MIN_INTERVAL_MS 3000 // pause after init before the first read, datasheet needs >= 2000 


static const char *TAG = "am2320";


// crc16 as in datasheet, i2c address is not included
static uint16_t am2320_crc16 (const uint8_t *buf, size_t len) {
	uint16_t crc = AM2320_CRC_INIT;

	while (len--) {
		crc ^= *buf++;
		for (int i = 0; i < 8; i++) {
			if (crc & 0x01) {
				crc = (crc >> 1) ^ AM2320_CRC_POLY;
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}


// wake + read command + read answer, returns only register bytes in out
// (the bus handle is not needed any more: shared_i2c_probe() knows the bus itself)
static esp_err_t am2320_read_regs (i2c_master_dev_handle_t dev, uint8_t start_reg, uint8_t count, uint8_t *out) {
	uint8_t read_cmd[3] = {AM2320_CMD_READ_REGS, start_reg, count};
	uint8_t data[AM2320_HEADER_LEN + AM2320_MAX_REGS + AM2320_CRC_LEN];
	size_t data_len = AM2320_HEADER_LEN + count + AM2320_CRC_LEN;
	esp_err_t err;

	if (count == 0 || count > AM2320_MAX_REGS) {
		return ESP_ERR_INVALID_ARG;
	}

	// (1) wake up sensor: address only, sensor does not ACK, so result is ignored
	(void) shared_i2c_probe(AM2320_ADDR); // (void) -> result is ignored on purpose 
	vTaskDelay(pdMS_TO_TICKS(AM2320_DELAY_WAKE_MS));

	// (2) send read command: function code + start register + number of registers
	err = shared_i2c_write(dev, read_cmd, sizeof(read_cmd));
	if (err != ESP_OK) {
		return err;
	}

	// (3) wait until the sensor prepares the answer
	vTaskDelay(pdMS_TO_TICKS(AM2320_DELAY_CMD_MS));

	// (4) read answer: function code + byte count + registers + CRC
	err = shared_i2c_read(dev, data, data_len);
	if (err != ESP_OK) {
		return err;
	}

	// (5) check header: must repeat our function code and count
	if (data[0] != AM2320_CMD_READ_REGS || data[1] != count) {
		return ESP_ERR_INVALID_RESPONSE;
	}

	// (6) check CRC: received low byte first
	uint16_t crc_received = ((uint16_t)data[data_len - 1] << 8) | data[data_len - 2];
	uint16_t crc_calculated = am2320_crc16(data, data_len - AM2320_CRC_LEN);
	if (crc_received != crc_calculated) {
		return ESP_ERR_INVALID_CRC;
	}

	// (7) copy register bytes without header and CRC
	for (uint8_t i = 0; i < count; i++) {
		out[i] = data[AM2320_HEADER_LEN + i];
	}

	return ESP_OK;
}


esp_err_t am2320_init (i2c_master_dev_handle_t *dev) {
	esp_err_t err;

	// (1) wait for sensor to wake up
	vTaskDelay(pdMS_TO_TICKS(AM2320_DELAY_POWER_ON_MS));

	// (2) adding device; the bus is already created in app_main 
	err = shared_i2c_add_device(AM2320_ADDR, AM2320_SCL_SPEED_HZ, dev);
	if (err != ESP_OK) {
		return err;
	}

	// (3) check if am2320 answers: read model, version and device ID
	uint8_t info[AM2320_INFO_REG_COUNT];

	err = am2320_read_regs(*dev, AM2320_REG_MODEL_HIGH, AM2320_INFO_REG_COUNT, info);
	if (err != ESP_OK) {
		return err;
	}
	uint16_t model = ((uint16_t)info[0] << 8) | info[1];
	uint8_t version = info[2];
	uint32_t device_id = ((uint32_t)info[3] << 24) | ((uint32_t)info[4] << 16) | ((uint32_t)info[5] << 8) | info[6];
	ESP_LOGI(TAG, "model: 0x%04X | version: 0x%02X | id: 0x%08lX", model, version, (unsigned long)device_id);

	// (4) every communication starts a new measurement, so keep the minimum interval before the first read
	vTaskDelay(pdMS_TO_TICKS(AM2320_MIN_INTERVAL_MS));

	return ESP_OK;
}


esp_err_t am2320_read (i2c_master_dev_handle_t dev, float *hum, float *temp) {
	uint8_t data[AM2320_DATA_REG_COUNT];
	esp_err_t err;

	// (1) wake + command + read 4 data registers (CRC is checked inside)
	err = am2320_read_regs(dev, AM2320_REG_HUM_HIGH, AM2320_DATA_REG_COUNT, data);
	if (err != ESP_OK) {
		return err;
	}

	// (2) combine bytes
	// hum = 16 bits -> data[0] (high), data[1] (low)
	// temp = 16 bits -> data[2] (high), data[3] (low), bit15 = sign
	uint16_t raw_hum = ((uint16_t)data[0] << 8) | data[1];
	uint16_t raw_temp = ((uint16_t)data[2] << 8) | data[3];

	// (3) convert to real values
	*hum = raw_hum / AM2320_RAW_DIVIDER;
	*temp = (raw_temp & AM2320_TEMP_VALUE_MASK) / AM2320_RAW_DIVIDER;
	if (raw_temp & AM2320_TEMP_SIGN_BIT) {
		*temp = -*temp;
	}

	return ESP_OK;
}