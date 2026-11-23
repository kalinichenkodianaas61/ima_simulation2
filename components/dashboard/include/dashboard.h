#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

// Terminal dashboard: units save their data here, a separate task redraws the screen in place.
// The dashboard knows nothing about sensors: main.c describes the layout (what and how to show).


#define DASH_MAX_UNITS 4 // max number of units (sections) on the screen 
#define DASH_ROWS_PER_UNIT 2 // sensors per unit 



// how to print the values of a unit 
typedef enum {
	DASH_VALUES_ACCEL, // v0, v1, v2 = gX, gY, gZ in g
	DASH_VALUES_BARO, // v0 = pressure in hPa, v1 = altitude in m 
	DASH_VALUES_HUM, // v0 = humidity in %, v1 = temperature in C 
} dash_values_t;

// description of one unit 
typedef struct {
	const char *title; // section title, e.g. "[1] ACCEL" 
	int period_ms; // shown next to the title 
	dash_values_t values; // how to print values 
	const char *sensor_name[DASH_ROWS_PER_UNIT]; // NULL -> row is not used 
	int dp[DASH_ROWS_PER_UNIT]; // decimal places per sensor 
} dash_unit_cfg_t;


// start the dashboard task and turn off ESP_LOG (the dashboard owns the screen now)
// units must point to an array that lives forever (static const in main.c)
// call before unit tasks start, otherwise their first data is lost
esp_err_t dashboard_start (const dash_unit_cfg_t *units, int unit_count, uint32_t period_ms, UBaseType_t priority);

// save init result of sensor (unit, row); does nothing if the dashboard is not started
void dashboard_save_init (int unit, int row, esp_err_t err);

// save read result; on error the old values stay on the screen
void dashboard_save_read (int unit, int row, esp_err_t err, float v0, float v1, float v2);











