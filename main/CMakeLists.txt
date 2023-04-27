idf_component_register(
  SRCS main.cpp lcd.cpp esp_logo.c esp_text.c lvgl_demo_ui.c
  INCLUDE_DIRS .
  REQUIRES lvgl esp_lcd driver esp_lcd_touch_gt911
)
