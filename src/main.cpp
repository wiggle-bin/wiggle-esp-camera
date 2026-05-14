#include <Arduino.h>
#include "env.h"
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include "esp_camera.h"
#include <esp_sleep.h>
#include "image_transmission.h"

ImageTransmissionState imageTransmissionState;
ImageTransmissionConfig imageTransmissionConfig = {RECEIVER_MAC, 9, 0};

// Pin connected to 2N2222A base (GPIO3) to turn off power to LED ring via NPN transistor
#define NPN_TRANSISTOR_PIN 3

// LED ring
#define LED_PIN 44 // Adjust if using a different GPIO
#define NUM_LEDS 8

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Camera configuration
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM

// === Feature Toggles ===
#define LIGHT_ALWAYS_ON false       // Set to true to keep LED ring on (soft white)
#define READ_SENSORS_MINUTES 0.2      // Time in minutes between sensor readings
#define READ_SENSORS_INTERVAL (READ_SENSORS_MINUTES * 60UL * 1000UL) // Time in ms

// === Energy saving Toggles ===
#define DEEP_SLEEP false             // Set to true deep sleep between sensor readings and photo captures
#define DEEP_SLEEP_MINUTES 0.2       // Deep sleep duration in minutes
#define DEEP_SLEEP_INTERVAL_US (DEEP_SLEEP_MINUTES * 60ULL * 1000000ULL)

#include "camera_pins.h"

// Utility: generate a timestamp string (optional)
String getTimestampedFilename()
{
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "esp32_%Y%m%d_%H%M%S.jpg", timeinfo);
  return String(buffer);
}

WiFiClient espClient;

unsigned long lastCaptureTime = 0; // Last shooting time
int imageCount = 1;                // File Counter
bool camera_sign = false;          // Check camera status
bool sd_sign = false;              // Check sd status

// Timing variables
unsigned long setupStartTime = 0;
unsigned long setupEndTime = 0;
unsigned long loopStartTime = 0;
unsigned long loopEndTime = 0;

void turnOnLedRing(uint8_t r = 255, uint8_t g = 0, uint8_t b = 0, uint8_t brightness = 255)
{
  strip.setBrightness(brightness);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

void turnOffLedRing()
{
  strip.clear();
  strip.show();
}

void photo_save(const char *fileName, const bool sd_sign)
{
  // Turn on LED ring before taking the photo
  turnOnLedRing(); // Default: white at brightness 30
  delay(1000);     // Give time for illumination

  sensor_t* sensor = esp_camera_sensor_get();
  // To turn sensor out of deel sleep mode. Have to check if this is actually needed.
  // Based on form https://forum.seeedstudio.com/t/xiao-esp32s3-sense-camera-sleep-current/271258/32?page=3
  if (sensor && DEEP_SLEEP) {
     sensor->set_reg(sensor, 0x3008, 0x40, 0x00);
  }

  camera_fb_t *fb = esp_camera_fb_get(); // dummy read
  if (fb)
    esp_camera_fb_return(fb); // return dummy

  delay(200); // short wait before real capture

  fb = esp_camera_fb_get(); // real capture

  if (!fb)
  {
    Serial.println("Failed to get camera frame buffer");
    if (LIGHT_ALWAYS_ON)
    {
      // Soft white, low brightness (e.g., 64 out of 255)
      turnOnLedRing(255, 255, 255, 64);
    }
    else
    {
      turnOffLedRing();
    }
    return;
  }

  // Post photo
  // sendPhoto(fb);

  // Release image buffer
  esp_camera_fb_return(fb);

  // Power down camera sensor after use to save energy
  if (sensor && DEEP_SLEEP) {
    sensor->set_reg(sensor, 0x3008, 0x40, 0x40);
  }

  delay(1000);
  if (LIGHT_ALWAYS_ON)
  {
    // Soft white, low brightness (e.g., 64 out of 255)
    turnOnLedRing(255, 255, 255, 64);
  }
  else
  {
    turnOffLedRing();
  }
}

// === Globals ===
int photoCount = 0;


void setup() {
  setupStartTime = millis();
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  initializeBaseImagePersistence(imageTransmissionState);

  // // Turn on power to LED ring via PNP transistor
  // pinMode(NPN_TRANSISTOR_PIN, OUTPUT);
  // digitalWrite(NPN_TRANSISTOR_PIN, HIGH); 

  if (LIGHT_ALWAYS_ON)
  {
    // Soft white, low brightness (e.g., 64 out of 255)
    turnOnLedRing(255, 255, 255, 64);
  }

  // === ESP-NOW ===
  initializeESPNowSender(imageTransmissionConfig);

  // === Camera Init ===
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  // Always capture a grayscale QQVGA frame for diffing
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QQVGA; // 160x120
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  // Adjustments for image in darker environement
  s->set_brightness(s, 2);  // -2 to 2
  s->set_contrast(s, 1);    // -2 to 2
  s->set_saturation(s, -1); // -2 to 2 (less color noise)

  s->set_gain_ctrl(s, 1);     // Auto gain on
  s->set_exposure_ctrl(s, 1); // Auto exposure on
  s->set_awb_gain(s, 1);      // Auto white balance

  s->set_agc_gain(s, 30);    // Higher = brighter (0 to 30+)
  s->set_aec_value(s, 1200); // Exposure time (0–1200+)

  s->set_vflip(s, 1); // If your image is upside down

  camera_sign = true; // Camera initialization check passes

  setupEndTime = millis();
}

void loop() {
  loopStartTime = millis();

  // === Conditional image capture and send ===

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb || fb->len != IMAGE_TRANSMISSION_BASE_IMAGE_SIZE) {
    Serial.println("Camera capture failed or unexpected size");
    if (fb) esp_camera_fb_return(fb);
    goto sleep;
  }

  processFrameAndSendIfChanged(fb->buf, imageTransmissionState, imageTransmissionConfig);

  esp_camera_fb_return(fb);

sleep:

  loopEndTime = millis();
  unsigned long setupTime = setupEndTime - setupStartTime;
  unsigned long loopTime = loopEndTime - loopStartTime;
  unsigned long totalTime = loopEndTime - setupStartTime;
  Serial.printf("[TIMING] setup: %lu ms, loop: %lu ms, total: %lu ms\n", setupTime, loopTime, totalTime);

  if (DEEP_SLEEP)
  {
    // Turn LED ring power off via NPN transistor to save energy during deep sleep
    digitalWrite(NPN_TRANSISTOR_PIN, LOW);  // NPN OFF
    digitalWrite(LED_PIN, HIGH);
    // Turn off WiFi and Bluetooth to minimize power consumption
    // Have to check if this is actually needed
    WiFi.mode(WIFI_OFF);
    btStop();
    // Enter Deep sleep
    Serial.printf("Entering deep sleep for %.2f minutes...\n", (double)DEEP_SLEEP_INTERVAL_US / 60000000.0);
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_INTERVAL_US);
    Serial.flush();
    esp_deep_sleep_start();
    // Code never reaches here until wakeup
  }
  else
  {
    Serial.println("Waiting 1 minute...");
    delay(READ_SENSORS_INTERVAL); // 1 minute
  }
}