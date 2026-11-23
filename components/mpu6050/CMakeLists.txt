# sensor driver uses the shared bus 
idf_component_register(SRCS "mpu6050.c"
                       INCLUDE_DIRS "include"
                       REQUIRES shared_i2c)
