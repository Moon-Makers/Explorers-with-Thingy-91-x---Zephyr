cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(BLE_NRF9151)
# FILE(GLOB app_sources src/*.c)

# Clean simple main for nRF5340 binary protocol
# target_sources(app PRIVATE src/main_clean.c)
# NORDIC SDK APP START
# target_sources(app PRIVATE ${app_sources})

# target_sources(app PRIVATE src/gps_coap.c)
target_sources(app PRIVATE src/mqtt_connection.c)
target_sources(app PRIVATE src/main_clean.c)
# NORDIC SDK APP END
