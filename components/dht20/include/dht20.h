#pragma once 

#include "esp_err.h"
#include "shared_i2c.h"

#define DHT20_DP_NUMBER 1 // decimal places for printing


// wait after power-on, add sensor to the shared bus, check calibration bits
esp_err_t dht20_init (i2c_master_dev_handle_t *dev);

// humidity in % and temperature in C; blocks ~90 ms; call not more often then every 2s
esp_err_t dht20_read (i2c_master_dev_handle_t dev, float *hum, float *temp);