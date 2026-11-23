# sensor driver uses the shared bus 
idf_component_register(SRCS "am2320.c"
                       INCLUDE_DIRS "include"
                       REQUIRES shared_i2c)
