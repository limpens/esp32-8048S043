ESP32-8048S043C
----

Sunton ESP32-S3 800x480 Capacitive touch display


Example using esp-idf 5.1 and the esp_lcd_touch_gt911 and lvgl components.

In gt911_touch_init, a callback is registered to map the measured touch coordinates to
display coordinates, see header file for information.

When calling LCDInit(), the display, touch sensor and lvgl buffers are initialized.

The code will print a increasing number in the right-bottom corner and run the demo once.
Tapping the screen will change the text color of this number.

* Set esp-idf target to ESP32S3, other versions might lack rgb panel support.
* The supplied sdkconfig.defaults configures SPIRAM, regenerate your sdkconfig if needed.


idf.py set-target esp32s3
idf.py build flash monitor
