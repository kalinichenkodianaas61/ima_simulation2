#pragma once

#include "esp_err.h"
#include "shared_i2c.h"

#define MPU6050_DP_NUMBER 2 // decimal places for printing 

// add sensor to the shared bus, check WHO_AM_I, wake up, set +-2g
esp_err_t mpu6050_init (i2c_master_dev_handle_t *dev);

// read acceleration in g 
esp_err_t mpu6050_read (i2c_master_dev_handle_t dev, float *gX, float *gY, float *gZ);
