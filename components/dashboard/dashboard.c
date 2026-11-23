#include <stdio.h>
#include <stdbool.h>
#include <string.h> // memcpy()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "dashboard.h"

// task settings
#define DASH_STACK_SIZE 4096 // bytes, printf with floats needs a bigger stack 

// layout 
#define DASH_VALUES_WIDTH 40 // width of the values column in characters 

// ANSI terminal codes: special character sequences the terminal executes instead of printing
#define ANSI_HOME "\033[H" // cursor to top-left corner 
#define ANSI_CLEAR_SCREEN "\033[2J" // clear whole screen 
#define ANSI_CLEAR_LINE_END "\033[K" // clear from cursor to end of line 
#define ANSI_CLEAR_BELOW "\033[J" // clear from cursor to end of screen 
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_RED "\033[31m"
#define ANSI_RESET "\033[0;32m" // reset style, normal color is green 


// state of one sensor 
typedef enum {
	SENSOR_STARTING = 0, // init is not finished yet 
	SENSOR_OK, // = 1, last read was good 
	SENSOR_READ_FAILED, // = 2, last read failed, values are from an older read 
	SENSOR_INIT_FAILED, // = 3, sensor did not start, it is not used 
} sensor_status_t;

typedef struct {
	sensor_status_t status;
	esp_err_t err; // last error code 
	float val[3];
	uint32_t reads; // number of good reads, shows that data is alive 
} sensor_state_t;


// static -> only this file sees the data, units change it only through dashboard_save_*()
static sensor_state_t s_state[DASH_MAX_UNITS][DASH_ROWS_PER_UNIT]; // zero at start -> SENSOR_STARTING 
static SemaphoreHandle_t s_lock = NULL; // protects s_state 
static const dash_unit_cfg_t *s_units = NULL; // layout from main.c
static int s_unit_count = 0;
static uint32_t s_period_ms = 0;




// ========= SAVING DATA ==========

// true if (unit, row) exists and the dashboard is running 
static bool dash_place_ok (int unit, int row) {
	return s_lock != NULL
		&& unit >= 0 && unit < s_unit_count
		&& row >= 0 && row < DASH_ROWS_PER_UNIT;
}


void dashboard_save_init (int unit, int row, esp_err_t err) {
	if (!dash_place_ok(unit, row)) {
		return;
	}

	// lock is held for microseconds and nothing inside can hang, so waiting forever is safe
	xSemaphoreTake(s_lock, portMAX_DELAY);
	sensor_state_t *s = &s_state[unit][row];
	s->err = err;
	if (err == ESP_OK) {
		s->status = SENSOR_OK;
	} else {
		s->status = SENSOR_INIT_FAILED;
	}
	xSemaphoreGive(s_lock);
}


void dashboard_save_read (int unit, int row, esp_err_t err, float v0, float v1, float v2) {
	if (!dash_place_ok(unit, row)) {
		return;
	}

	xSemaphoreTake(s_lock, portMAX_DELAY);
	sensor_state_t *s = &s_state[unit][row];
	s->err = err;
	if (err == ESP_OK) {
		s->status = SENSOR_OK;
		s->val[0] = v0;
		s->val[1] = v1;
		s->val[2] = v2;
		s->reads++;
	} else {
		s->status = SENSOR_READ_FAILED;
	}
	xSemaphoreGive(s_lock);
}




// ========= DRAWING ==========

// status column: colored word + read counter 
static void print_status (const sensor_state_t *s) {
	switch (s->status) {
	case SENSOR_STARTING:
		printf(ANSI_YELLOW "starting..." ANSI_RESET);
		break;
	case SENSOR_OK:
		printf(ANSI_GREEN "OK" ANSI_RESET "  #%lu", (unsigned long)s->reads);
		break;
	case SENSOR_READ_FAILED:
		printf(ANSI_RED "READ FAIL: %s" ANSI_RESET "  #%lu", esp_err_to_name(s->err), (unsigned long)s->reads);
		break;
	case SENSOR_INIT_FAILED:
		printf(ANSI_RED "INIT FAIL: %s" ANSI_RESET, esp_err_to_name(s->err));
		break;
	}
	printf(ANSI_CLEAR_LINE_END "\n"); // erase leftovers of a longer old line 
}


// one row: name | values | status 
static void print_row (const char *name, dash_values_t kind, int dp, const sensor_state_t *s) {
	char values[DASH_VALUES_WIDTH + 16];

	// (1) values into a buffer: snprintf writes text into memory instead of the screen
	if (s->reads == 0) {
		snprintf(values, sizeof(values), "---"); // no good read yet
	} else {
		switch (kind) {
		case DASH_VALUES_ACCEL:
			snprintf(values, sizeof(values), "X %+6.*f   Y %+6.*f   Z %+6.*f  g",
				dp, s->val[0], dp, s->val[1], dp, s->val[2]);
			break;
		case DASH_VALUES_BARO:
			snprintf(values, sizeof(values), "%8.*f hPa   %8.*f m", dp, s->val[0], dp, s->val[1]);
			break;
		case DASH_VALUES_HUM:
			snprintf(values, sizeof(values), "%6.*f %%     %+7.*f C", dp, s->val[0], dp, s->val[1]);
			break;
		}
	}

	// (2) print with fixed column widths, so statuses stand one under another
	// %-9s -> left-aligned in 9 chars, %-*s -> width is taken from the argument
	printf("    %-9s%-*s", name, DASH_VALUES_WIDTH, values);
	print_status(s);
}


// dashboard task: every s_period_ms redraws the screen from the top
static void dashboard_task (void *arg) {
	(void) arg; // not used 
	sensor_state_t snap[DASH_MAX_UNITS][DASH_ROWS_PER_UNIT]; // local copy 

	printf(ANSI_CLEAR_SCREEN); // once: remove boot messages 

	TickType_t last_wake = xTaskGetTickCount();
	while (1) {
		// (1) copy everything quickly under the lock; arrays cannot be assigned with "=", so memcpy
		xSemaphoreTake(s_lock, portMAX_DELAY);
		memcpy(snap, s_state, sizeof(snap));
		xSemaphoreGive(s_lock);

		// (2) draw outside the lock: printing is slow, units must not wait for it
		unsigned long up_ms = (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS);

		printf(ANSI_HOME ANSI_RESET); // draw over the previous frame in green
		printf("  "  "======== IMA demo ========" ANSI_RESET "   uptime %lu.%lu s" ANSI_CLEAR_LINE_END "\n",
			up_ms / 1000, (up_ms % 1000) / 100);

		for (int u = 0; u < s_unit_count; u++) {
			const dash_unit_cfg_t *cfg = &s_units[u];

			printf(ANSI_CLEAR_LINE_END "\n"); // empty line between units 
			printf("  "  "%s" ANSI_RESET "  (every %d ms)" ANSI_CLEAR_LINE_END "\n", cfg->title, cfg->period_ms);

			for (int r = 0; r < DASH_ROWS_PER_UNIT; r++) {
				if (cfg->sensor_name[r] != NULL) {
					print_row(cfg->sensor_name[r], cfg->values, cfg->dp[r], &snap[u][r]);
				}
			}
		}

		printf(ANSI_CLEAR_BELOW); // erase anything below the frame 
		fflush(stdout); // send everything to the terminal now 

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(s_period_ms));
	}
}




// ========= START ==========

esp_err_t dashboard_start (const dash_unit_cfg_t *units, int unit_count, uint32_t period_ms, UBaseType_t priority) {
	// (1) check arguments and that it was not started before 
	if (s_lock != NULL) {
		return ESP_ERR_INVALID_STATE;
	}
	if (units == NULL || unit_count <= 0 || unit_count > DASH_MAX_UNITS || period_ms == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	// (2) remember the layout 
	s_units = units;
	s_unit_count = unit_count;
	s_period_ms = period_ms;

	// (3) mutex for the data 
	s_lock = xSemaphoreCreateMutex();
	if (s_lock == NULL) {
		return ESP_ERR_NO_MEM;
	}

	// (4) drawing task 
	if (xTaskCreate(dashboard_task, "dashboard", DASH_STACK_SIZE, NULL, priority, NULL) != pdPASS) {
		vSemaphoreDelete(s_lock); // roll back, as in shared_i2c_init 
		s_lock = NULL;
		return ESP_ERR_NO_MEM;
	}

	// (5) the dashboard owns the screen: turn off ESP_LOG for all tags ("*"), errors are shown on the panel
	esp_log_level_set("*", ESP_LOG_NONE);

	return ESP_OK;
}







