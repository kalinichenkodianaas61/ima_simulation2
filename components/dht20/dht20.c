#include <stdint.h> 
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h" 
#include "esp_log.h" 
#include "shared_i2c.h"
#include "dht20.h"


// i2c
#define DHT20_ADDR 0x38 // 7-bit i2c address
#define DHT20_SCL_SPEED_HZ 400000 // 100kHz -> slow mode 

// commands 
#define DHT20_CMD_MEASURE 0xAC
#define DHT20_CMD_MEASURE_ARG1 0x33 // 1st argument of measure command
#define DHT20_CMD_MEASURE_ARG2 0x00 // 2st argument of measure command

// status byte
#define DHT20_STATUS_BUSY 0x80 // bit[7]: 1 - busy, 0 - idle
#define DHT20_STATUS_CAL_MASK 0x18 // bits[4:3]: both 1 -> calibrated

// data layout 
#define DHT20_DATA_LEN 7 // status + 5 data bytes + CRC
#define DHT20_NIBBLE_MASK 0x0F // keeps lower 4 bits

// conversion
#define DHT20_RAW_FULL_SCALE 1048576.0f // 2^20, max value of 20-bit reading
#define DHT20_HUM_SCALE 100.0f // humidity range in %
#define DHT20_TEMP_SCALE 200.0f // temperature range in Celsius
#define DHT20_TEMP_OFFSET 50.0f // temperature offset in Celsius

// delays in ms
#define DHT20_DELAY_POWER_ON_MS 300 // wait after power on, datasheet needs >= 100
#define DHT20_DELAY_BEFORE_CMD_MS 10 // pause before trigger command
#define DHT20_DELAY_MEASURE_MS 80 // measure time


static const char *TAG = "dht20";


esp_err_t dht20_init (i2c_master_dev_handle_t *dev) {
	esp_err_t err;
	
	// (1) wait for sensor to wake up
	vTaskDelay(pdMS_TO_TICKS(DHT20_DELAY_POWER_ON_MS));
	
	// (2) adding device 
	err = shared_i2c_add_device(DHT20_ADDR, DHT20_SCL_SPEED_HZ, dev);
	
	// (3) check if dht20 is calibrated
	uint8_t calibrate_output = 0;

	err = shared_i2c_read(*dev, &calibrate_output, 1);
	if (err != ESP_OK) {
		return err;
	}
	if ((calibrate_output & DHT20_STATUS_CAL_MASK) == DHT20_STATUS_CAL_MASK) {
		ESP_LOGI(TAG, "calibrate output: 0x%02X (expected 0x%02X)", calibrate_output, DHT20_STATUS_CAL_MASK);
	} else {
		ESP_LOGW(TAG, "calibrate output: 0x%02X (expected 0x%02X)", calibrate_output, DHT20_STATUS_CAL_MASK);
	}

	return ESP_OK;
	
}


esp_err_t dht20_read (i2c_master_dev_handle_t dev, float *hum, float *temp) {
	uint8_t trigger_reading[3] = {DHT20_CMD_MEASURE, DHT20_CMD_MEASURE_ARG1, DHT20_CMD_MEASURE_ARG2};
	uint8_t data[DHT20_DATA_LEN];
	esp_err_t err;

	// (1) trigger measurement
	vTaskDelay(pdMS_TO_TICKS(DHT20_DELAY_BEFORE_CMD_MS));
	err = shared_i2c_write(dev, trigger_reading, sizeof(trigger_reading));
	if (err != ESP_OK) {
		return err;
	}

	// (2) wait until the sensor finishes measuring; the bus is free for other tasks meanwhile
	vTaskDelay(pdMS_TO_TICKS(DHT20_DELAY_MEASURE_MS));

	// (3) read 7 bytes: status + 5 data bytes + CRC
	err = shared_i2c_read(dev, data, DHT20_DATA_LEN);
	if (err != ESP_OK) {
		return err;
	}

	// (4) check if sensor busy
	if (data[0] & DHT20_STATUS_BUSY) {
		return ESP_ERR_INVALID_STATE;
	}

	// (5) combine bits
	// hum = 20 bits -> data[1], data[2], high nibble of data[3]
	// temp = 20 bits -> low nibble of data[3], data[4], data[5]
	uint32_t raw_hum = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)data[3] >> 4);
	uint32_t raw_temp = ((uint32_t)(data[3] & DHT20_NIBBLE_MASK) << 16) | ((uint32_t)data[4] << 8) | ((uint32_t)data[5]);

	// (6) convert to real values
	*hum = raw_hum / DHT20_RAW_FULL_SCALE * DHT20_HUM_SCALE;
	*temp = raw_temp / DHT20_RAW_FULL_SCALE * DHT20_TEMP_SCALE - DHT20_TEMP_OFFSET;

	return ESP_OK;
	
}










 