# esp_driver_i2c gives driver/i2c_master.h (ESP-IDF >= 5.3)

idf_component_register(SRCS "shared_i2c.c"
						INCLUDE_DIRS "include"
						REQUIRES esp_driver_i2c)