include(px4_add_board)

px4_add_board(
    PLATFORM        nuttx
    TOOLCHAIN       arm-none-eabi
    ARCHITECTURE    cortex-m33
    CONSTRAINED_FLASH
    CONSTRAINED_MEMORY

    ROMFSROOT       px4fmu_common

    IO
        px4_io-v2_default

    SERIAL_PORTS
        GPS1:/dev/ttyS2
        TEL1:/dev/ttyS0

    DRIVERS
        barometer/bmp280
        imu/invensense/mpu6500
        magnetometer/hmc5883
        gps
        px4io
        adc/board_adc

    MODULES
        sensors
        ekf2
        commander
        mc_att_control
        mc_rate_control
        control_allocator
        rc_update
        manual_control
        land_detector
        events
        battery_status
        load_mon
        mavlink
        logger
        dataman
        gyro_calibration

    SYSTEMCMDS
        param
        nshterm
        reboot
        dmesg
        perf
        top
        work_queue
        uorb
        led_control
        tune_control
        ver
)