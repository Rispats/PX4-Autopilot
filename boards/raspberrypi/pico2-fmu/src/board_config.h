#pragma once

#include <nuttx/config.h>

#define BOARD_NUMBER_I2C_BUSES          2   // I2C0=BARO, I2C1=MAG   
#define BOARD_NUMBER_SPI_BUSES          2   // SPI0=IMU, SPI1=SD

// SPI
#define PX4_SPI_BUS_IMU                 0    // SPI0=IMU MPU6500
#define PX4_SPI_BUS_MEMORY              1    // SPI1=SD card
#define PX4_SPIDEV_IMU                  1    // CS = GP5
#define PX4_SPIDEV_MEMORY               2    // CS = GP13

// GPIO pin numbers for bit-banged CS
#define BOARD_SPI0_CS_PIN               (5)  // GP5 - IMU CS
#define BOARD_SPI1_CS_PIN               (13) // GP13 - SD CS
#define BOARD_IMU_DRDY_PIN             (6)  // GP6 - MPU6500 INT pin

// I2C
#define PX4_I2C_BUS_ONBOARD             0    // I2C0 → BMP280 baro  (GP8/GP9)
#define PX4_I2C_BUS_EXPANSION           1    // I2C1 → HMC5883 mag  (GP14/GP15)

// UART0 — optional telemetry or NSH debug shell
#define TELEM1_SERIAL_PORT              "/dev/ttyS0"   // GP0/GP1

// UART1 — PX4IO coprocessor (STM32F103 Dev board)
#define PX4IO_SERIAL_DEVICE             "/dev/ttyS1"   // GP16/GP17
#define PX4IO_SERIAL_BAUDRATE           1500000

// PIO UART — GPS (NuttX rp23xx PIO UART driver)
#define GPS_DEFAULT_UART_PORT           "/dev/ttyS2"   // GP18/GP19 via PIO
#define GPS_DEFAULT_BAUDRATE            9600           // adjust to your GPS module default

// RP2350 PWM slices for RGB LED 
#define BOARD_PWM_LED_R_PIN             (20)        //GP20=slice2A
#define BOARD_PWM_LED_G_PIN             (21)        //GP21=slice2B
#define BOARD_PWM_LED_B_PIN             (22)        //GP22=slice3A
#define BOARD_HAS_LED_PWM               1
#define BOARD_LED_PWM_DRIVE_ACTIVE_LOW  0           // set 1 if your LED is common-anode

#define BOARD_SAFETY_BUTTON_PIN         (26)        // GP26, active low, internal pull-up
#define BOARD_SAFETY_BUTTON_POLARITY    0           // 0 = pressed when LOW

#define ADC_BATTERY_VOLTAGE_CHANNEL     2           // GP28 = ADC2
#define BOARD_ADC_SCALE_VOLTAGE         (3.3f / 4096.0f * BOARD_VOLTAGE_DIVIDER_RATIO)
#define BOARD_VOLTAGE_DIVIDER_RATIO     (5.7f)      // 10k/2k voltage divider for 3S battery

// USB
#define BOARD_HAS_USB_CDC               1
#define BOARD_USB_CDC_PORT              "/dev/ttyACM0"

#define BOARD_HAS_RC_SERIAL
#define BOARD_OVERRIDE_UUID             "pico2-fmu-poc-v1"