ESP32-8048S043C
----

Sunton ESP32-S3 800x480 Capacitive touch display


Example using esp-idf and the esp_lcd_touch_gt911 component to run lvgl on the display.

In gt911_touch_init, a callback is registered to map the measured touch coordinates to
display coordinates, see header file for information.

When calling LCDInit(), the display, touch sensor and lvgl buffers are initialized.

The code will print a increasing number in the right-bottom corner and run the demo once.

BUG: The touch sensor sometimes throws errors due to i2c communication although the
component is issueing a reset. 

Set esp-idf target to ESP32S3.
