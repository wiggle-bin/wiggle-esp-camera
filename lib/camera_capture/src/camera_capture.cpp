#include "camera_capture.h"

#include <Arduino.h>

namespace {

// XIAO ESP32S3 camera pins.
constexpr int PIN_D0 = 15;
constexpr int PIN_D1 = 17;
constexpr int PIN_D2 = 18;
constexpr int PIN_D3 = 16;
constexpr int PIN_D4 = 14;
constexpr int PIN_D5 = 12;
constexpr int PIN_D6 = 11;
constexpr int PIN_D7 = 48;
constexpr int PIN_XCLK = 10;
constexpr int PIN_PCLK = 13;
constexpr int PIN_VSYNC = 38;
constexpr int PIN_HREF = 47;
constexpr int PIN_SCCB_SDA = 40;
constexpr int PIN_SCCB_SCL = 39;
constexpr int PIN_PWDN = -1;
constexpr int PIN_RESET = -1;

} // namespace

bool initializeCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = PIN_D0;
  config.pin_d1 = PIN_D1;
  config.pin_d2 = PIN_D2;
  config.pin_d3 = PIN_D3;
  config.pin_d4 = PIN_D4;
  config.pin_d5 = PIN_D5;
  config.pin_d6 = PIN_D6;
  config.pin_d7 = PIN_D7;
  config.pin_xclk = PIN_XCLK;
  config.pin_pclk = PIN_PCLK;
  config.pin_vsync = PIN_VSYNC;
  config.pin_href = PIN_HREF;
  config.pin_sccb_sda = PIN_SCCB_SDA;
  config.pin_sccb_scl = PIN_SCCB_SCL;
  config.pin_pwdn = PIN_PWDN;
  config.pin_reset = PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("Camera sensor handle unavailable");
    return false;
  }

  // Disable auto controls for stable, repeatable diff frames.
  s->set_gain_ctrl(s, 0);
  s->set_exposure_ctrl(s, 0);
  s->set_awb_gain(s, 0);

  // Fixed exposure and gain values.
  s->set_aec_value(s, 300);
  s->set_agc_gain(s, 8);

  // Additional image tuning.
  s->set_brightness(s, 2);
  s->set_contrast(s, 1);
  s->set_saturation(s, -2);

  return true;
}

camera_fb_t *capturePhotoFrame(bool deepSleepEnabled, bool useDummyRead, size_t expectedFrameSize) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor && deepSleepEnabled) {
    sensor->set_reg(sensor, 0x3008, 0x40, 0x00);
  }

  if (useDummyRead) {
    camera_fb_t *dummy = esp_camera_fb_get();
    if (dummy) {
      esp_camera_fb_return(dummy);
    }
    delay(200);
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (sensor && deepSleepEnabled) {
    sensor->set_reg(sensor, 0x3008, 0x40, 0x40);
  }

  if (!fb) {
    Serial.println("Failed to get camera frame buffer");
    return nullptr;
  }

  if (fb->len != expectedFrameSize) {
    Serial.println("Camera capture failed or unexpected size");
    esp_camera_fb_return(fb);
    return nullptr;
  }

  return fb;
}