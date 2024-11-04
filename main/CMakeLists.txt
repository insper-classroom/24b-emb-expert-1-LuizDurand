add_executable(main
        main.c
        mpu6050.c
)

target_link_libraries(main pico_stdlib hardware_i2c freertos Fusion)
pico_add_extra_outputs(main)

add_library(mpu6050 mpu6050.c)
