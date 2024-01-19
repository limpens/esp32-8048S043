#include "esp_log.h"
#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt911.h" // from managed component
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "lvgl.h"

#include "lcd.h"

#define TAG "display"

const i2c_port_t i2c_master_port = I2C_NUM_0;
static SemaphoreHandle_t xGuiSemaphore = NULL;
static TaskHandle_t g_lvgl_task_handle;

static void lcd_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    auto panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

esp_err_t InitI2C(void)
{
i2c_config_t conf = {
  .mode       = I2C_MODE_MASTER,
  .sda_io_num = TOUCH_PIN_SDA,
  .scl_io_num = TOUCH_PIN_SCL,
  .sda_pullup_en = GPIO_PULLUP_ENABLE,
  .scl_pullup_en = GPIO_PULLUP_ENABLE,
  .master
  {
    .clk_speed = TOUCH_FREQ_HZ,
  },
  .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL
};

  ESP_LOGI(TAG, "Initializing I2C");

  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}


uint16_t map(uint16_t n, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t  out_max)
{
  return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void process_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
  //
  // in gt911_touchpad_read we ask for a single (1) measurement, so we do not loop over all points.
  //
  *x = map(*x, TOUCH_H_RES_MIN, TOUCH_H_RES_MAX, 0, LCD_H_RES);
  *y = map(*y, TOUCH_V_RES_MIN, TOUCH_V_RES_MAX, 0, LCD_V_RES);
}

void gt911_touch_init(esp_lcd_touch_handle_t *tp)
{
esp_lcd_panel_io_handle_t tp_io_handle = nullptr;

const esp_lcd_panel_io_i2c_config_t tp_io_config = { .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS,
  .on_color_trans_done = nullptr,
  .user_ctx = nullptr,
  .control_phase_bytes = 1,
  .dc_bit_offset = 0,
  .lcd_cmd_bits = 16,
  .lcd_param_bits = 0,
  .flags = {
    .dc_low_on_data = 0,
    .disable_control_phase = 1,
  } };

const esp_lcd_touch_config_t tp_cfg = {
  .x_max = LCD_H_RES,
  .y_max = LCD_V_RES,
  .rst_gpio_num = TOUCH_PIN_RESET,
  .int_gpio_num = TOUCH_PIN_INT,
  .levels = {
      .reset = 0,
      .interrupt = 0,
  },
  .flags = {
      .swap_xy = 0,
      .mirror_x = 0,
      .mirror_y = 0,
  },
  .process_coordinates = process_coordinates, // callback to fix coordinates between gt911 and display
  .interrupt_callback = nullptr
};

  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)i2c_master_port, &tp_io_config, &tp_io_handle));
  ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, tp));
}

static void gt911_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
  auto tp = (esp_lcd_touch_handle_t)indev_drv->user_data;
  assert(tp);

  uint16_t touchpad_x;
  uint16_t touchpad_y;
  uint8_t touchpad_cnt = 0;

  esp_lcd_touch_read_data(tp);

  bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, &touchpad_x, &touchpad_y, nullptr, &touchpad_cnt, 1);
  if (touchpad_pressed && touchpad_cnt > 0)
  {
    data->point.x = touchpad_x;
    data->point.y = touchpad_y;
    data->state = LV_INDEV_STATE_PRESSED;
  }
  else 
  {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

extern "C" void lvgl_acquire(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (g_lvgl_task_handle != task)
    {
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    }
}

extern "C" void lvgl_release(void)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    if (g_lvgl_task_handle != task)
    {
        xSemaphoreGive(xGuiSemaphore);
    }
}

void lvUpdateTask(void *)
{
  while(true)
  {
    vTaskDelay(pdMS_TO_TICKS(20));

    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
    {
      lv_task_handler();
      xSemaphoreGive(xGuiSemaphore);
    }
  }
}


esp_err_t LCDInit(void)
{
static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // contains callback functions
static lv_indev_drv_t indev_drv_tp;
static esp_lcd_touch_handle_t tp;
static esp_lcd_panel_handle_t panel_handle = nullptr;

gpio_config_t bk_gpio_config = {
  .pin_bit_mask = 1ULL << LCD_PIN_BK_LIGHT,
  .mode         = GPIO_MODE_OUTPUT,
  .pull_up_en   = GPIO_PULLUP_DISABLE,
  .pull_down_en = GPIO_PULLDOWN_DISABLE,
  .intr_type    = GPIO_INTR_DISABLE
};
esp_lcd_rgb_panel_config_t panel_config = {
  .clk_src = LCD_CLK_SRC_DEFAULT, // PLL_F160M
  .timings = {
    .pclk_hz  = LCD_PIXEL_CLOCK_HZ,
    .h_res    = LCD_H_RES,
    .v_res    = LCD_V_RES,

    .hsync_pulse_width  = 4,
    .hsync_back_porch   = 8,
    .hsync_front_porch  = 8,
    .vsync_pulse_width  = 4,
    .vsync_back_porch   = 8,
    .vsync_front_porch  = 8,
    .flags{
      .hsync_idle_low   = false,
      .vsync_idle_low   = false,
      .de_idle_high     = false,
      .pclk_active_neg  = true,
      .pclk_idle_high   = false
    }
  },
  .data_width             = 16,
  .bits_per_pixel         = 0,
  .num_fbs                = 2,
  .bounce_buffer_size_px  = 0, //4 * LCD_H_RES,
  .sram_trans_align       = 0,
  .psram_trans_align      = 64,

  .hsync_gpio_num         = LCD_PIN_HSYNC,
  .vsync_gpio_num         = LCD_PIN_VSYNC,
  .de_gpio_num            = LCD_PIN_DE,
  .pclk_gpio_num          = LCD_PIN_PCLK,
  .disp_gpio_num          = LCD_PIN_DISP_EN,
  .data_gpio_nums = {
    LCD_PIN_DATA0,
    LCD_PIN_DATA1,
    LCD_PIN_DATA2,
    LCD_PIN_DATA3,
    LCD_PIN_DATA4,
    LCD_PIN_DATA5,
    LCD_PIN_DATA6,
    LCD_PIN_DATA7,
    LCD_PIN_DATA8,
    LCD_PIN_DATA9,
    LCD_PIN_DATA10,
    LCD_PIN_DATA11,
    LCD_PIN_DATA12,
    LCD_PIN_DATA13,
    LCD_PIN_DATA14,
    LCD_PIN_DATA15
  },
  .flags
  {
    .disp_active_low      = 0,
    .refresh_on_demand    = 0,
    .fb_in_psram          = true,
    .double_fb            = true,
    .no_fb                = 0,
    .bb_invalidate_cache  = 0
  }
};

  ESP_LOGI(TAG, "Turn off LCD backlight");
  ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

  ESP_LOGI(TAG, "Install RGB LCD panel driver");
  ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

#if SYNC
  esp_lcd_rgb_panel_event_callbacks_t cbs = {
      .on_vsync = example_on_vsync_event,
  };
  ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));
#endif

  ESP_LOGI(TAG, "Initialize RGB LCD panel");
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

  ESP_LOGI(TAG, "Initialize LVGL library");
  lv_init();

  ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
  void *buf1 = heap_caps_malloc(LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf1);
  void *buf2 = heap_caps_malloc(LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
  assert(buf2);

  //
  // initialize LVGL draw buffers
  //
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 100);

  ESP_LOGI(TAG, "Register display driver to LVGL");
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res    = LCD_H_RES;
  disp_drv.ver_res    = LCD_V_RES;
  disp_drv.flush_cb   = lcd_lvgl_flush_cb;
  disp_drv.draw_buf   = &disp_buf;
  disp_drv.user_data  = panel_handle;

  lv_disp_drv_register(&disp_drv);

  //
  // setup/configure input device:
  //
  ESP_ERROR_CHECK(InitI2C());
  gt911_touch_init(&tp);

  // Register a touchpad input device
  lv_indev_drv_init(&indev_drv_tp);
  indev_drv_tp.type = LV_INDEV_TYPE_POINTER;
  indev_drv_tp.read_cb = gt911_touchpad_read;
  indev_drv_tp.user_data = tp;

  xGuiSemaphore = xSemaphoreCreateMutex();

  if (lv_indev_drv_register(&indev_drv_tp))
  {
    xTaskCreatePinnedToCore(&lvUpdateTask, "lv_update", 4096, nullptr, tskIDLE_PRIORITY, &g_lvgl_task_handle, 1);
    return ESP_OK;
  }

  return ESP_FAIL;
}
