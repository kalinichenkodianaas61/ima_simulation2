#pragma once // include this header only once per .c file

#include <stdint.h> 
#include <stddef.h> 
#include "esp_err.h" 
#include "driver/i2c_master.h"

// One i2c bus shared by all sensors. Every function below takes a mutex, 
// so sensors can be read from different FreeRTOS tasks at the same time.


// create the bus and it`s mutex; call once in app_main before tasks start
esp_err_t shared_i2c_init(i2c_port_num_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

// add a device with 7-bit address to the bus
esp_err_t shared_i2c_add_device(uint16_t addr, uint32_t scl_speed_hz, i2c_master_dev_handle_t *dev);

// write len (length) bytes
esp_err_t shared_i2c_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len);

// read len (length) bytes
esp_err_t shared_i2c_read(i2c_master_dev_handle_t dev, uint8_t *data, size_t len);

// write, the read; usually "register address -> register values"
esp_err_t shared_i2c_write_read(i2c_master_dev_handle_t dev, const uint8_t *wdata, size_t wlen, uint8_t *rdata, size_t rlen);

// address only: ESP_OK if the device answered ACK, ESP_ERR_NOT_FOUND on NACK
esp_err_t shared_i2c_probe(uint16_t addr);





