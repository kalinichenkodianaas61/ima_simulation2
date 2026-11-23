#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h" 

// components
#include "shared_i2c.h"
#include "dashboard.h"
#include "mpu6050.h"
#include "mma8452.h"
#include "lps331ap.h"
#include "bmp388.h"
#include "am2320.h"
#include "dht20.h"

// i2c
#define BOARD_I2C_PORT I2C_NUM_0
#define BOARD_I2C_SDA_GPIO GPIO_NUM_4
#define BOARD_I2C_SCL_GPIO GPIO_NUM_5

// i2c bus map, all addresses must be different 
// 0x1C mma8452 | 0x38 dht20 | 0x5C am2320 | 0x5D lps331ap | 0x68 mpu6050 | 0x77 bmp388

// output mode: 1 -> dashboard redrawn in place, 0 -> plain scrolling log
#define USE_DASHBOARD 1
#define DASHBOARD_PERIOD_MS 250 // screen refresh period 

// unit periods in ms 
#define UNIT_ACCEL_PERIOD_MS 250
#define UNIT_BARO_PERIOD_MS 500
#define UNIT_HUM_PERIOD_MS 3000 // am2320 and dht20 need >= 2000

// task settings
#define UNIT_STACK_SIZE 4096 // bytes, printing floats needs a bigger stack
#define UNIT_ACCEL_PRIORITY 5 // the fastest unit gets the highest priority 
#define UNIT_BARO_PRIORITY 4
#define UNIT_HUM_PRIORITY 3
#define DASHBOARD_PRIORITY 2 // drawing is the least important job


// ========== AXIS ALIGNMENT ==========

// how a sensor is mounted: which of its axes gives common X, Y, Z, and with which sign
// common axes = mpu6050 axes 

typedef struct {
	uint8_t src[3]; // sensor axis for common X, Y, Z: 0 = X, 1 = Y, 2 = Z 
	int8_t sign[3]; // +1 same direction, -1 opposite
} axis_map_t;

// mpu6050 is the reference
static const axis_map_t MPU6050_AXES = { .src = {0, 1, 2}, .sign = {+1, +1, +1} };

// mma8452 is turned 90 deg: common X = -mma Y, common Y = +mma X, common Z = +mma Z
static const axis_map_t MMA8452_AXES = { .src = {1, 0, 2}, .sign = {-1, +1, +1} };


// convert x, y, z from sensor axes to common axes 
static void axis_apply (const axis_map_t *m, float *x, float *y, float *z) {
	float in[3] = {*x, *y, *z}; // copy first, x/y/z are overwritten below 
	*x = m->sign[0] * in[m->src[0]];
	*y = m->sign[1] * in[m->src[1]];
	*z = m->sign[2] * in[m->src[2]];
}


// ========== DASHBOARD LAYOUT ==========

// unit numbers = section numbers on the dashboard 
enum {
	UNIT_ACCEL = 0,
	UNIT_BARO,
	UNIT_HUM,
	UNIT_COUNT, // = 3, always last 
};

// row of each sensor inside its unit, same order as sensor_name below
#define ROW_MPU6050 0
#define ROW_MMA8452 1
#define ROW_LPS331 0
#define ROW_BMP388 1
#define ROW_AM2320 0
#define ROW_DHT20 1

// static const: lives the whole program and cannot be changed; [UNIT_ACCEL] = ... puts the item at that index
static const dash_unit_cfg_t s_layout[UNIT_COUNT] = {
	[UNIT_ACCEL] = {
		.title = "ACCEL",
		.period_ms = UNIT_ACCEL_PERIOD_MS,
		.values = DASH_VALUES_ACCEL,
		.sensor_name = {"mpu6050", "mma8452"},
		.dp = {MPU6050_DP_NUMBER, MMA8452_DP_NUMBER},
	},
	[UNIT_BARO] = {
		.title = "BARO",
		.period_ms = UNIT_BARO_PERIOD_MS,
		.values = DASH_VALUES_BARO,
		.sensor_name = {"lps331ap", "bmp388"},
		.dp = {LPS331_DP_NUMBER, BMP388_DP_NUMBER},
	},
	[UNIT_HUM] = {
		.title = "TEMP / HUM",
		.period_ms = UNIT_HUM_PERIOD_MS,
		.values = DASH_VALUES_HUM,
		.sensor_name = {"am2320", "dht20"},
		.dp = {AM2320_DP_NUMBER, DHT20_DP_NUMBER},
	},
};



// ========== UNITS ==========

// log + save init result to the dashboard, return true if the sensor works
static bool unit_sensor_ok (const char *tag, int unit, int row, esp_err_t err) {
	dashboard_save_init(unit, row, err);
	const char *sensor = s_layout[unit].sensor_name[row];

	if (err == ESP_OK) {
		ESP_LOGI(tag, "%s ready", sensor);
		return true;
	}
	ESP_LOGE(tag, "%s init failed: %s", sensor, esp_err_to_name(err));
	return false;
}


// unit 1: accelerometers mpu6050 + mma8452
static void unit_accel_task (void *arg) {
	(void) arg; // not used 
	const char *tag = "unit_accel";
	i2c_master_dev_handle_t mpu = NULL;
	i2c_master_dev_handle_t mma = NULL;

	// (1) init both sensors; the unit keeps working if at least one of them is ok
	bool mpu_ok = unit_sensor_ok(tag, UNIT_ACCEL, ROW_MPU6050, mpu6050_init(&mpu));
	bool mma_ok = unit_sensor_ok(tag, UNIT_ACCEL, ROW_MMA8452, mma8452_init(&mma));
	if (!mpu_ok && !mma_ok) {
		ESP_LOGE(tag, "no working sensors, unit stopped");
		vTaskDelete(NULL); // NULL -> delete this task 
	}


	// (2) read in loop with a fixed period 
	TickType_t last_wake = xTaskGetTickCount();
	
	while (1) {
		float gX = 0, gY = 0, gZ = 0;
		esp_err_t err;

		if (mpu_ok) {
			err = mpu6050_read(mpu, &gX, &gY, &gZ);
			if (err == ESP_OK) {
				axis_apply(&MPU6050_AXES, &gX, &gY, &gZ); // to common axes 
			}
			dashboard_save_read(UNIT_ACCEL, ROW_MPU6050, err, gX, gY, gZ);
			if (err == ESP_OK) {
				ESP_LOGI(tag, "mpu6050 | gX: %+7.*f | gY: %+7.*f | gZ: %+7.*f",
					MPU6050_DP_NUMBER, gX,
					MPU6050_DP_NUMBER, gY,
					MPU6050_DP_NUMBER, gZ);
			} else {
				ESP_LOGW(tag, "mpu6050 read failed: %s", esp_err_to_name(err));
			}
		}

		if (mma_ok) {
			err = mma8452_read(mma, &gX, &gY, &gZ);
			if (err == ESP_OK) {
				axis_apply(&MMA8452_AXES, &gX, &gY, &gZ); // to common axes 
			}
			dashboard_save_read(UNIT_ACCEL, ROW_MMA8452, err, gX, gY, gZ);
			if (err == ESP_OK) {
				ESP_LOGI(tag, "mma8452 | gX: %+7.*f | gY: %+7.*f | gZ: %+7.*f",
					MMA8452_DP_NUMBER, gX,
					MMA8452_DP_NUMBER, gY,
					MMA8452_DP_NUMBER, gZ);
			} else {
				ESP_LOGW(tag, "mma8452 read failed: %s", esp_err_to_name(err));
			}
		}

		// wake up exactly one period after the previous wake-up, no matter how long reading took
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(UNIT_ACCEL_PERIOD_MS));
	}
}




// unit 2: barometers lps331ap + bmp388 
static void unit_baro_task (void *arg) {
	(void) arg; // not used 
	const char *tag = "unit_baro";
	i2c_master_dev_handle_t lps = NULL;
	i2c_master_dev_handle_t bmp = NULL;
	bmp388_calib_t bmp_calib; // calibration lives as long as the task 

	// (1) init both sensors 
	bool lps_ok = unit_sensor_ok(tag, UNIT_BARO, ROW_LPS331, lps331_init(&lps));
	bool bmp_ok = unit_sensor_ok(tag, UNIT_BARO, ROW_BMP388, bmp388_init(&bmp, &bmp_calib));
	if (!lps_ok && !bmp_ok) {
		ESP_LOGE(tag, "no working sensors, unit stopped");
		vTaskDelete(NULL);
	}

	// (2) read in loop with a fixed period 
	// 1 mbar = 1 hPa, so both barometers print in hPa 
	TickType_t last_wake = xTaskGetTickCount();
	while (1) {
		float press = 0, alt = 0;
		esp_err_t err;

		if (lps_ok) {
			err = lps331_read(lps, &press, &alt);
			dashboard_save_read(UNIT_BARO, ROW_LPS331, err, press, alt, 0);
			if (err == ESP_OK) {
				ESP_LOGI(tag, "lps331ap | press: %7.*f hPa | alt: %7.*f m",
					LPS331_DP_NUMBER, press,
					LPS331_DP_NUMBER, alt);
			} else {
				ESP_LOGW(tag, "lps331ap read failed: %s", esp_err_to_name(err));
			}
		}

		if (bmp_ok) {
			err = bmp388_read(bmp, &bmp_calib, &press, &alt);
			dashboard_save_read(UNIT_BARO, ROW_BMP388, err, press, alt, 0);
			if (err == ESP_OK) {
				ESP_LOGI(tag, "bmp388   | press: %7.*f hPa | alt: %7.*f m",
					BMP388_DP_NUMBER, press,
					BMP388_DP_NUMBER, alt);
			} else {
				ESP_LOGW(tag, "bmp388 read failed: %s", esp_err_to_name(err));
			}
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(UNIT_BARO_PERIOD_MS));
	}
}




// unit 3: temperature/humidity am2320 + dht20 
static void unit_hum_task (void *arg) {
	(void) arg; // not used 
	const char *tag = "unit_hum";
	i2c_master_dev_handle_t am = NULL;
	i2c_master_dev_handle_t dht = NULL;

	// (1) init both sensors; am2320 init takes ~5 s, other units do not wait for it
	bool dht_ok = unit_sensor_ok(tag, UNIT_HUM, ROW_DHT20, dht20_init(&dht));
	bool am_ok = unit_sensor_ok(tag, UNIT_HUM, ROW_AM2320, am2320_init(&am));
	if (!am_ok && !dht_ok) {
		ESP_LOGE(tag, "no working sensors, unit stopped");
		vTaskDelete(NULL);
	}

	// (2) read in loop with a fixed period 
	TickType_t last_wake = xTaskGetTickCount();
	while (1) {
		float hum = 0, temp = 0;
		esp_err_t err;

		if (am_ok) {
			err = am2320_read(am, &hum, &temp);
			dashboard_save_read(UNIT_HUM, ROW_AM2320, err, hum, temp, 0);
			if (err == ESP_OK) {
				ESP_LOGI(tag, "am2320 | hum: %6.*f %% | temp: %+7.*f C",
					AM2320_DP_NUMBER, hum,
					AM2320_DP_NUMBER, temp);
			} else {
				ESP_LOGW(tag, "am2320 read failed: %s", esp_err_to_name(err));
			}
		}

		if (dht_ok) {
			err = dht20_read(dht, &hum, &temp);
			dashboard_save_read(UNIT_HUM, ROW_DHT20, err, hum, temp, 0);
			if (err == ESP_OK) {
				ESP_LOGI(tag, "dht20  | hum: %6.*f %% | temp: %+7.*f C",
					DHT20_DP_NUMBER, hum,
					DHT20_DP_NUMBER, temp);
			} else {
				ESP_LOGW(tag, "dht20 read failed: %s", esp_err_to_name(err));
			}
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(UNIT_HUM_PERIOD_MS));
	}
}




// create a task, report if it failed 
static void start_task (TaskFunction_t fn, const char *name, UBaseType_t priority) {
	// xTaskCreate(function, name, stack in bytes, argument, priority, handle)
	if (xTaskCreate(fn, name, UNIT_STACK_SIZE, NULL, priority, NULL) != pdPASS) {
		printf("cannot create task %s\n", name); // printf: works even when logs are off 
	}
}


void app_main(void) {
	// (1) one bus for all units, created before any task start
	ESP_ERROR_CHECK(shared_i2c_init(BOARD_I2C_PORT, BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO));

#if USE_DASHBOARD
	// (2) dashboard before units, so it gets their very first data
	ESP_ERROR_CHECK(dashboard_start(s_layout, UNIT_COUNT, DASHBOARD_PERIOD_MS, DASHBOARD_PRIORITY));
#endif

	// (3) start three units, each one is an independent task
	start_task(unit_accel_task, "unit_accel", UNIT_ACCEL_PRIORITY);
	start_task(unit_baro_task, "unit_baro", UNIT_BARO_PRIORITY);
	start_task(unit_hum_task, "unit_hum", UNIT_HUM_PRIORITY);

	// app_main may return here, the tasks keep running
}









