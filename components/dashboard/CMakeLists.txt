# dashboard needs only FreeRTOS and log, they are always available 
idf_component_register(SRCS "dashboard.c"
                       INCLUDE_DIRS "include")