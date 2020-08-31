cmake_minimum_required(VERSION 3.17)
project(c_modbus C)

set(CMAKE_C_STANDARD 99)

add_executable(c_modbus
        main.c
        modbus/modbus.c
        modbus/modbus.h
        modbus/modbus_config.h
        soft_timer/soft_timer.c
        soft_timer/soft_timer.h

#        utility/lz_assert.c
        utility/lz_utils.c
        uart/uart.c
        uart/uart.h
        )

find_package(Threads)
target_link_libraries(c_modbus ${CMAKE_THREAD_LIBS_INIT})