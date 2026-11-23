#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h" // for mutex
#include "hal/i2c_types.h"
#include "soc/clk_tree_defs.h"
#include "soc/dport_access.h"
#include "soc/gpio_num.h"
#include <stdint.h>
#include "shared_i2c.h"

// bus settings
#define SHARED_I2C_GLITCH_IGNORE_CNT 7 // default value

// timeouts in ms
#define SHARED_I2C_XFER_TIMEOUT_MS 100 // one i2c transaction 
#define SHARED_I2C_LOCK_TIMEOUT_MS 500 // max wait while another task uses the bus

// static -> visible only inside this file, other files use the functions before  
static i2c_master_bus_handle_t s_bus = NULL;
static SemaphoreHandle_t s_lock = NULL; // mutex descriptor


// take the mutex -> only one task talks on the bus at a time
static esp_err_t shared_i2c_lock (void) {
	if (s_lock == NULL) {
		return ESP_ERR_INVALID_STATE; // bus was not initialized, shared_i2c_init() was not called
	}
	
	if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(SHARED_I2C_LOCK_TIMEOUT_MS)) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}
	
	return ESP_OK;
}


// give the mutex back, for another task
static void shared_i2c_unlock (void) {
	xSemaphoreGive(s_lock);
}


// bus initialization
esp_err_t shared_i2c_init (i2c_port_num_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio) {
	esp_err_t err;
	
	// (1) the bus can be created only once
	if (s_lock != NULL) {
		return ESP_ERR_INVALID_STATE;
	}
	
	// (2) create mutex; must be created before any task begin to run
	s_lock = xSemaphoreCreateMutex();
	if (s_lock == NULL) { // after successful creation s_lock is no longer NULL
		return ESP_ERR_NO_MEM;
	}
	
	// (3) create bus
	i2c_master_bus_config_t bus_cfg = {
		.i2c_port = port,
		.sda_io_num = sda_gpio,
		.scl_io_num = scl_gpio,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = SHARED_I2C_GLITCH_IGNORE_CNT, 
		.flags.enable_internal_pullup = true, 
	};
	err = i2c_new_master_bus(&bus_cfg, &s_bus);
	if (err != ESP_OK) {
		vSemaphoreDelete(s_lock); // frees the mutex memory
		s_lock = NULL; // deletes the old mutex address
		s_bus = NULL;
		return err;
	}
	
	return ESP_OK;
	
}


// 
esp_err_t shared_i2c_add_device (uint16_t addr, uint32_t scl_speed_hz, i2c_master_dev_handle_t *dev) {
	esp_err_t err;
		
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = addr,
		.scl_speed_hz = scl_speed_hz,
	};
	
	err = shared_i2c_lock(); // to avoid race condition
	if (err != ESP_OK) {
		return err;
	}
	err = i2c_master_bus_add_device(s_bus, &dev_cfg, dev);
	shared_i2c_unlock();
	
	return err;
	
}


esp_err_t shared_i2c_write (i2c_master_dev_handle_t dev, const uint8_t *data, size_t len) {
	esp_err_t err;
	
	err = shared_i2c_lock();
	if (err !=  ESP_OK) {
		return err; 
	}
	err = i2c_master_transmit(dev, data, len, SHARED_I2C_XFER_TIMEOUT_MS);
	shared_i2c_unlock();
	
	return err;
	
}


esp_err_t shared_i2c_read (i2c_master_dev_handle_t dev, uint8_t *data, size_t len) {
	esp_err_t err;
	
	err = shared_i2c_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = i2c_master_receive (dev, data, len, SHARED_I2C_XFER_TIMEOUT_MS);
	shared_i2c_unlock();
	
	return err;
	
}


esp_err_t shared_i2c_write_read (i2c_master_dev_handle_t dev, const uint8_t *wdata, size_t wlen,
																uint8_t *rdata, size_t rlen) {
	esp_err_t err;
	
	err = shared_i2c_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = i2c_master_transmit_receive(dev, wdata, wlen, rdata, rlen, SHARED_I2C_XFER_TIMEOUT_MS);
	shared_i2c_unlock();
	
	return err;
	
}


esp_err_t shared_i2c_probe (uint16_t addr) {
	esp_err_t err;
	
	err = shared_i2c_lock();
	if (err != ESP_OK) {
		return err;
	}
	err = i2c_master_probe(s_bus, addr, SHARED_I2C_XFER_TIMEOUT_MS);
	shared_i2c_unlock();
	
	return err;
	
}














