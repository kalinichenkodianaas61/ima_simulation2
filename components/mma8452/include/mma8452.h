#pragma once

#include "esp_err.h"
#include "shared_i2c.h"

#define MMA8452_DP_NUMBER 2 // decimal places for printing 

// add sensor to the shared bus, check WHO_AM_I, set +-2g and 100Hz, switch to active mode
esp_err_t mma8452_init (i2c_master_dev_handle_t *dev);

// read acceleration in g 
esp_err_t mma8452_read (i2c_master_dev_handle_t dev, float *gX, float *gY, float *gZ);
