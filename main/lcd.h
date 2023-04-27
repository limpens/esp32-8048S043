#pragma once

#include "esp_err.h"

//
// hardware configuration for the ESP32-8048S043 board
//

#define LCD_H_RES 800
#define LCD_V_RES 480

//
// min / max determined by printing x/y coordinates while pressing different locations on the display
// can be reproduced by printing the numbers in process_coordinates before calling the map function
//
#define TOUCH_H_RES_MIN 0
#define TOUCH_H_RES_MAX 477
#define TOUCH_V_RES_MIN 0
#define TOUCH_V_RES_MAX 269

//
// Serious distortion going above 18Mhz:
//
#define LCD_PIXEL_CLOCK_HZ     (18 * 1000 * 1000)

//
// GPIO numbers determined from ESP32-8048S043-1.png
//
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define LCD_PIN_BK_LIGHT      (gpio_num_t)GPIO_NUM_2  // 

#define LCD_PIN_HSYNC         (gpio_num_t)GPIO_NUM_39 // HSYNC
#define LCD_PIN_VSYNC         (gpio_num_t)GPIO_NUM_41 // VSYNC
#define LCD_PIN_DE            (gpio_num_t)GPIO_NUM_40 // DE
#define LCD_PIN_PCLK          (gpio_num_t)GPIO_NUM_42 // DCLK
#define LCD_PIN_DATA0         (gpio_num_t)GPIO_NUM_8  // B3
#define LCD_PIN_DATA1         (gpio_num_t)GPIO_NUM_3  // B4
#define LCD_PIN_DATA2         (gpio_num_t)GPIO_NUM_46 // B5
#define LCD_PIN_DATA3         (gpio_num_t)GPIO_NUM_9  // B6
#define LCD_PIN_DATA4         (gpio_num_t)GPIO_NUM_1  // B7
#define LCD_PIN_DATA5         (gpio_num_t)GPIO_NUM_5  // G2
#define LCD_PIN_DATA6         (gpio_num_t)GPIO_NUM_6  // G3
#define LCD_PIN_DATA7         (gpio_num_t)GPIO_NUM_7  // G4
#define LCD_PIN_DATA8         (gpio_num_t)GPIO_NUM_15 // G5
#define LCD_PIN_DATA9         (gpio_num_t)GPIO_NUM_16 // G6
#define LCD_PIN_DATA10        (gpio_num_t)GPIO_NUM_4  // G7
#define LCD_PIN_DATA11        (gpio_num_t)GPIO_NUM_45 // R3
#define LCD_PIN_DATA12        (gpio_num_t)GPIO_NUM_48 // R4
#define LCD_PIN_DATA13        (gpio_num_t)GPIO_NUM_47 // R5
#define LCD_PIN_DATA14        (gpio_num_t)GPIO_NUM_21 // R6
#define LCD_PIN_DATA15        (gpio_num_t)GPIO_NUM_14 // R7
#define LCD_PIN_DISP_EN       (gpio_num_t)GPIO_NUM_NC // not connected

#define TOUCH_PIN_RESET       (gpio_num_t)GPIO_NUM_38 // REST
#define TOUCH_PIN_SCL         (gpio_num_t)GPIO_NUM_20
#define TOUCH_PIN_SDA         (gpio_num_t)GPIO_NUM_19
#define TOUCH_FREQ_HZ         (400000)

esp_err_t LCDInit(void);
