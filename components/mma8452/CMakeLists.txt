# sensor driver uses the shared bus 
idf_component_register(SRCS "mma8452.c"
                       INCLUDE_DIRS "include"
                       REQUIRES shared_i2c)
