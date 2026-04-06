#include <nuttx/config.h>

// SYSTEM CLOCKS 
#define BOARD_XOSC_FREQ                 12000000UL   // 12 MHz crystal on Pico2W
#define BOARD_PLL_SYS_FREQ              150000000UL  // 150 MHz system clock
#define BOARD_PLL_USB_FREQ              48000000UL   // 48 MHz USB PLL

// SPI0 — IMU MPU6500 
#define GPIO_SPI0_SCK                   (GPIO_FUNC_SPI | GPIO_PIN2)
#define GPIO_SPI0_MOSI                  (GPIO_FUNC_SPI | GPIO_PIN3)
#define GPIO_SPI0_MISO                  (GPIO_FUNC_SPI | GPIO_PIN4)
// CS is bit-banged GPIO, not hardware NSS
#define GPIO_SPI0_CS_IMU                (GPIO_OUTPUT | GPIO_PIN5)
#define BOARD_SPI0_FREQ                 8000000UL    // 8 MHz max for MPU6500

// SPI1 — SD CARD 
#define GPIO_SPI1_SCK                   (GPIO_FUNC_SPI | GPIO_PIN10)
#define GPIO_SPI1_MOSI                  (GPIO_FUNC_SPI | GPIO_PIN11)
#define GPIO_SPI1_MISO                  (GPIO_FUNC_SPI | GPIO_PIN12)
#define GPIO_SPI1_CS_SD                 (GPIO_OUTPUT | GPIO_PIN13)
#define BOARD_SPI1_FREQ                 25000000UL   // 25 MHz SD SPI mode max

// IMU INTERRUPT 
#define GPIO_IMU_DRDY                   (GPIO_INPUT | GPIO_PIN6)
#define BOARD_IMU_DRDY_PIN              6

// I2C0 — BMP280 BAROMETER 
#define GPIO_I2C0_SDA                   (GPIO_FUNC_I2C | GPIO_PIN8)
#define GPIO_I2C0_SCL                   (GPIO_FUNC_I2C | GPIO_PIN9)
#define BOARD_I2C0_FREQ                 400000UL     // 400 kHz fast mode

// I2C1 — HMC5883 MAGNETOMETER 
#define GPIO_I2C1_SDA                   (GPIO_FUNC_I2C | GPIO_PIN14)
#define GPIO_I2C1_SCL                   (GPIO_FUNC_I2C | GPIO_PIN15)
#define BOARD_I2C1_FREQ                 400000UL

// UART0 — TELEMETRY / NSH DEBUG (OPTIONAL) 
#define GPIO_UART0_TX                   (GPIO_FUNC_UART | GPIO_PIN0)
#define GPIO_UART0_RX                   (GPIO_FUNC_UART | GPIO_PIN1)
#define BOARD_UART0_BAUD                57600

// UART1 — PX4IO (STM32F103)
#define GPIO_UART1_TX                   (GPIO_FUNC_UART | GPIO_PIN16)
#define GPIO_UART1_RX                   (GPIO_FUNC_UART | GPIO_PIN17)
#define BOARD_UART1_BAUD                1500000      // PX4IO protocol speed

// PIO UART — GPS
// Uses PIO state machine 0 on RP2350
// NuttX rp23xx PIO UART driver binds this as /dev/ttyS2
#define GPIO_PIO_UART_TX                (GPIO_FUNC_PIO | GPIO_PIN18)
#define GPIO_PIO_UART_RX                (GPIO_FUNC_PIO | GPIO_PIN19)
#define BOARD_PIO_UART_BAUD             38400         // GPS module: uBlox M7N
#define BOARD_PIO_UART_SM               0            // PIO state machine index

// RGB LED (PWM) 
#define GPIO_PWM_LED_R                  (GPIO_FUNC_PWM | GPIO_PIN20)
#define GPIO_PWM_LED_G                  (GPIO_FUNC_PWM | GPIO_PIN21)
#define GPIO_PWM_LED_B                  (GPIO_FUNC_PWM | GPIO_PIN22)
#define BOARD_PWM_LED_SLICE_R           2            // PWM slice 2, channel A
#define BOARD_PWM_LED_SLICE_G           2            // PWM slice 2, channel B
#define BOARD_PWM_LED_SLICE_B           3            // PWM slice 3, channel A
#define BOARD_PWM_FREQ                  1000         // 1 kHz LED PWM frequency

// SAFETY / ARM BUTTON
#define GPIO_BTN_SAFETY                 (GPIO_INPUT | GPIO_PIN26)
#define BOARD_SAFETY_BUTTON_PIN         26
// Active LOW — pressed = GND, internal pull-up enabled in init.c

// BATTERY ADC
// GP28 = ADC2 — connect via voltage divider (e.g. 10k/2k for 3S)
#define GPIO_ADC_BATT_VOLT              (GPIO_FUNC_NULL | GPIO_PIN28)
#define BOARD_ADC_BATT_CHANNEL          2

// USB CDC
#define BOARD_HAS_USB_CDC               1
// USB D+/D- are internal on Pico2W — no GPIO defines needed

// RESERVED PINS (Pico2W — DO NOT USE)
// GP23 — CYW43439 WiFi WL_ON
// GP24 — CYW43439 SPI DATA / SDIO
// GP25 — CYW43439 SPI CS / LED
// GP29 — VSYS voltage sense (ADC3)

#endif /* __BOARDS_RPI_PICO2_FMU_INCLUDE_BOARD_H */