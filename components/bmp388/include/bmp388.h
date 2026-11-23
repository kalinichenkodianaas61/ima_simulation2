#pragma once

#include "esp_err.h"
#include "shared_i2c.h"

#define BMP388_DP_NUMBER 2 // decimal places for printing 

// calibration coefficients converted to float + t_lin for pressure compensation
// (in the header because main.c creates this struct and passes it to the functions)
typedef struct {
	float par_t1, par_t2, par_t3;
	float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
	float par_p7, par_p8, par_p9, par_p10, par_p11;
	float t_lin; // compensated temperature, needed by pressure formula
} bmp388_calib_t;

// add sensor to the shared bus, check chip id, soft reset, read calibration, set oversampling
esp_err_t bmp388_init (i2c_master_dev_handle_t *dev, bmp388_calib_t *calib);

// one forced-mode measurement: pressure in hPa and altitude in m; blocks ~50 ms
esp_err_t bmp388_read (i2c_master_dev_handle_t dev, bmp388_calib_t *calib, float *press, float *alt);
