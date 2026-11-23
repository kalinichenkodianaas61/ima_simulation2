#pragma once

#include "esp_err.h"
#include "shared_i2c.h"

#define AM2320_DP_NUMBER 1 // decimal places for printing, sensor resolution is 0.1 

// wait 2 s after power-on, add sensor to the shared bus, read model/version/id; blocks ~5 s
esp_err_t am2320_init (i2c_master_dev_handle_t *dev);

// humidity in % and temperature in C; call not more often than every 2 s
esp_err_t am2320_read (i2c_master_dev_handle_t dev, float *hum, float *temp);
