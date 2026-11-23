#pragma once

#include "esp_err.h"
#include "shared_i2c.h"

#define LPS331_DP_NUMBER 2 // decimal places for printing 

// add sensor to the shared bus, check WHO_AM_I, set averaging, power on in one-shot mode
esp_err_t lps331_init (i2c_master_dev_handle_t *dev);

// one measurement: pressure in mbar (= hPa) and altitude in m; blocks ~100 ms
esp_err_t lps331_read (i2c_master_dev_handle_t dev, float *press, float *alt);
