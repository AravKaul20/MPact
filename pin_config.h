#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ======================= PIN DEFINITIONS =======================

// Display Configuration
#define LCD_CS    10
#define LCD_DC    11
#define LCD_RST   12
#define LCD_BL    13
#define LCD_SCK   14
#define LCD_MOSI  15
#define LCD_WIDTH  240
#define LCD_HEIGHT 320

// I2C Configuration
#define IIC_SDA   17
#define IIC_SCL   18

// Touch Controller Configuration
#define TP_RST    16
#define TP_INT    21
#define CST816T_DEVICE_ADDRESS  0x15

// IMU Sensor Configuration
#define QMI8658_L_SLAVE_ADDRESS  0x6B

// Power Management Configuration
#define SYS_EN_PIN   35  // Power latch control
#define SYS_OUT_PIN  36  // Button input detection

// System Configuration
#define RESET_BUTTON_PIN  0  // GPIO0 for factory reset

#endif // PIN_CONFIG_H 