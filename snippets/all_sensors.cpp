#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "env.h"
#include <Wire.h>
#include <BH1750.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <EEPROM.h>

// === Image diff and conditional send ===
#define BASE_IMAGE_SIZE (160 * 120) // For FRAMESIZE_QQVGA, grayscale
#define DOWNSAMPLED_WIDTH 80
#define DOWNSAMPLED_HEIGHT 60
#define DOWNSAMPLED_IMAGE_SIZE (DOWNSAMPLED_WIDTH * DOWNSAMPLED_HEIGHT)
#define DOWNSAMPLED_PACKED_SIZE (DOWNSAMPLED_IMAGE_SIZE / 2) // 2 pixels per byte (4 bits each)
#define CHANGE_THRESHOLD 0         // Adjust for sensitivity


uint8_t baseImage[BASE_IMAGE_SIZE];
bool baseImageValid = false;
uint8_t downsampledImage[DOWNSAMPLED_IMAGE_SIZE];
uint8_t packedImage[DOWNSAMPLED_PACKED_SIZE];

// ESP-NOW peer MAC address (replace with your receiver's MAC)
uint8_t peerAddress[] = RECEIVER_MAC;

// Utility: generate a timestamp string (optional)
String getTimestampedFilename()
{
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "esp32_%Y%m%d_%H%M%S.jpg", timeinfo);
  return String(buffer);
}

// Debug: Send a simple "hello" ESP-NOW packet
void sendHelloESPNow() {
  const char *msg = "hello";
  esp_err_t result = esp_now_send(peerAddress, (const uint8_t *)msg, strlen(msg));
  if (result == ESP_OK) {
    Serial.println("ESP-NOW hello sent");
  } else {
    Serial.printf("ESP-NOW hello send failed: %d\n", result);
  }
}

// === Image Downsampling and Packing ===
// Downsample 160x120 grayscale to 80x60 (simple 2x2 average)
void downsample_160x120_to_80x60(const uint8_t *src, uint8_t *dst) {
  for (int y = 0; y < DOWNSAMPLED_HEIGHT; ++y) {
    for (int x = 0; x < DOWNSAMPLED_WIDTH; ++x) {
      int sum = 0;
      int src_x = x * 2;
      int src_y = y * 2;
      sum += src[(src_y + 0) * 160 + (src_x + 0)];
      sum += src[(src_y + 0) * 160 + (src_x + 1)];
      sum += src[(src_y + 1) * 160 + (src_x + 0)];
      sum += src[(src_y + 1) * 160 + (src_x + 1)];
      dst[y * DOWNSAMPLED_WIDTH + x] = sum / 4;
    }
  }
}

// Quantize 8-bit grayscale to 4-bit (0-15)
inline uint8_t quantize_4bit(uint8_t v) {
  return v >> 4; // 0-255 -> 0-15
}

// Pack 4-bit grayscale pixels (2 per byte)
void pack_4bit(const uint8_t *src, uint8_t *dst, int num_pixels) {
  for (int i = 0; i < num_pixels / 2; ++i) {
    uint8_t hi = quantize_4bit(src[i * 2]);
    uint8_t lo = quantize_4bit(src[i * 2 + 1]);
    dst[i] = (hi << 4) | (lo & 0x0F);
  }
}

// === ESP-NOW Transmission ===
void sendImageESPNow(const uint8_t *packed, size_t packed_len) {
  // ESP-NOW max payload is 250 bytes
  const size_t CHUNK_SIZE = 200; // Leave room for header
  uint16_t total_chunks = (packed_len + CHUNK_SIZE - 1) / CHUNK_SIZE;
  for (uint16_t chunk = 0; chunk < total_chunks; ++chunk) {
    size_t offset = chunk * CHUNK_SIZE;
    size_t len = (offset + CHUNK_SIZE > packed_len) ? (packed_len - offset) : CHUNK_SIZE;
    uint8_t buf[CHUNK_SIZE + 4];
    buf[0] = (uint8_t)(chunk & 0xFF);
    buf[1] = (uint8_t)((chunk >> 8) & 0xFF);
    buf[2] = (uint8_t)(total_chunks & 0xFF);
    buf[3] = (uint8_t)((total_chunks >> 8) & 0xFF);
    memcpy(buf + 4, packed + offset, len);
    esp_err_t result = esp_now_send(peerAddress, buf, len + 4);
    if (result == ESP_OK) {
      Serial.printf("ESP-NOW chunk %d/%d sent\n", chunk + 1, total_chunks);
    } else {
      Serial.printf("ESP-NOW send failed: %d\n", result);
    }
    delay(10); // Give time for radio
  }
}


// MQTT client (unused, but kept for compatibility)
WiFiClient espClient;
PubSubClient client(espClient);

// Light sensor
BH1750 lightMeter;

// DS18B20 setup
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Calibration and sensor config
// #define NUM_SENSORS 3
// float calibrationOffsets[NUM_SENSORS] = {0.0, 0.0, 0.0};

void addressToString(DeviceAddress deviceAddress, char *buffer)
{
  sprintf(buffer, "%02X%02X%02X%02X%02X%02X%02X%02X",
          deviceAddress[0], deviceAddress[1], deviceAddress[2], deviceAddress[3],
          deviceAddress[4], deviceAddress[5], deviceAddress[6], deviceAddress[7]);
}

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
#define TAKE_PICTURES true          // Set to false to disable all photo capture/sending
#define SMALL_IMAGE_SIZE false       // Set to true for smaller images to reduce battery drainage
#define DEEP_SLEEP true             // Set to true deep sleep between sensor readings and photo captures
#define DEEP_SLEEP_MINUTES 0.2       // Deep sleep duration in minutes
#define DEEP_SLEEP_INTERVAL_US (DEEP_SLEEP_MINUTES * 60ULL * 1000000ULL)

// === Storage Option ===
#define SAVE_TO_SD_CARD false // Set to true to save images to SD card

#include "camera_pins.h"


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

// SD card write file
void writeFile(fs::FS &fs, const char *path, uint8_t *data, size_t len)
{
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.write(data, len) == len)
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
  file.close();
}

// Save pictures to SD card
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
  
  if (!SAVE_TO_SD_CARD)
  {
    Serial.println("Skipping SD card save (SAVE_TO_SD_CARD = false)");
  }
  else if (SAVE_TO_SD_CARD && sd_sign)
  {
    // Save photo to file
    writeFile(SD, fileName, fb->buf, fb->len);
    Serial.println("Photo saved to file");
  }
  else
  {
    Serial.printf("sd_sign = %d\n", sd_sign);
  }

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


// ESP-NOW setup
// Set this to your receiver's WiFi channel (print WiFi.channel() on receiver after WiFi connects)
#define RECEIVER_WIFI_CHANNEL 9 // <-- CHANGE THIS to your actual WiFi channel

void setup_espnow() {
  WiFi.mode(WIFI_STA);
  // Set WiFi channel to match receiver
  esp_wifi_set_channel(RECEIVER_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.print("ESP-NOW sender using channel: ");
  Serial.println(RECEIVER_WIFI_CHANNEL);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(peerAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer");
    }
  }
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32SolarClient", mqtt_user, mqtt_password))
    {
      Serial.println(" connected.");
    }
    else
    {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5s.");
      delay(5000);
    }
  }
}

void setup() {
  setupStartTime = millis();
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Initialize EEPROM for base image persistence
  EEPROM.begin(BASE_IMAGE_SIZE + 1);
  baseImageValid = EEPROM.read(0) == 1;
  if (baseImageValid) {
    for (int i = 0; i < BASE_IMAGE_SIZE; ++i) baseImage[i] = EEPROM.read(i + 1);
  }

  // // Turn on power to LED ring via PNP transistor
  // pinMode(NPN_TRANSISTOR_PIN, OUTPUT);
  // digitalWrite(NPN_TRANSISTOR_PIN, HIGH); 

  if (LIGHT_ALWAYS_ON)
  {
    // Soft white, low brightness (e.g., 64 out of 255)
    turnOnLedRing(255, 255, 255, 64);
  }

  // === ESP-NOW ===
  setup_espnow();

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

  // #if SAVE_TO_SD_CARD
  //   // Initialize SD card
  //   if (!SD.begin(21))
  //   {
  //     Serial.println("Card Mount Failed");
  //   }
  //   uint8_t cardType = SD.cardType();

  //   // Determine if the type of SD card is available
  //   if (cardType == CARD_NONE)
  //   {
  //     Serial.println("No SD card attached");
  //   }

  //   Serial.print("SD Card Type: ");
  //   if (cardType == CARD_MMC)
  //   {
  //     Serial.println("MMC");
  //   }
  //   else if (cardType == CARD_SD)
  //   {
  //     Serial.println("SDSC");
  //   }
  //   else if (cardType == CARD_SDHC)
  //   {
  //     Serial.println("SDHC");
  //   }
  //   else
  //   {
  //     Serial.println("UNKNOWN");
  //   }

  //   sd_sign = true; // sd initialization check passes
  // #else
  //   sd_sign = false;
  // #endif

  // // Temp
  // sensors.begin();

  // // Start I2C on custom pins
  // Wire.begin(5, 6); // SDA = 5, SCL = 6

  // if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
  // {
  //   Serial.println("BH1750 sensor initialized");
  // }
  // else
  // {
  //   Serial.println("Error initializing BH1750 sensor. Check wiring.");
  // }

  setupEndTime = millis();
}

void loop() {
  loopStartTime = millis();

  // if (!client.connected())
  // {
  //   reconnect();
  // }

  // // Light sensor
  // float lux = lightMeter.readLightLevel();
  // Serial.print("Light: ");
  // Serial.print(lux);
  // Serial.println(" lx");
  // delay(1000);

  // // Temp sensors
  // sensors.requestTemperatures();
  // int deviceCount = sensors.getDeviceCount();
  // Serial.print("Found ");
  // Serial.print(deviceCount);
  // Serial.println(" DS18B20 sensor(s).");

  // for (int i = 0; i < deviceCount; i++)
  // {
  //   float tempC = sensors.getTempCByIndex(i);
  //   // tempC += calibrationOffsets[i];

  //   DeviceAddress sensorAddress;
  //   sensors.getAddress(sensorAddress, i);

  //   char sensorID[17];
  //   addressToString(sensorAddress, sensorID);

  //   Serial.print("Sensor ID ");
  //   Serial.print(sensorID);
  //   Serial.print(": ");
  //   Serial.print(tempC);
  //   Serial.println(" °C");

  //   char tempString[8];
  //   dtostrf(tempC, 6, 2, tempString);

  //   String topic = "home/sensors/" + String(sensorID);
  //   bool success = client.publish(topic.c_str(), tempString);

  //   if (success)
  //   {
  //     Serial.println("MQTT message published successfully.");
  //   }
  //   else
  //   {
  //     Serial.println("Failed to publish MQTT message.");

  //     if (!client.connected())
  //     {
  //       Serial.println("MQTT client not connected!");
  //     }

  //     Serial.print("MQTT client state: ");
  //     Serial.println(client.state());
  //   }
  // }

  // === Conditional image capture and send ===

  bool shouldSend = false;
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb || fb->len != BASE_IMAGE_SIZE) {
    Serial.println("Camera capture failed or unexpected size");
    if (fb) esp_camera_fb_return(fb);
    goto sleep;
  }

  if (!baseImageValid) {
    // First boot: store as base image
    memcpy(baseImage, fb->buf, BASE_IMAGE_SIZE);
    baseImageValid = true;
    Serial.println("Base image captured.");
    // Save to EEPROM
    EEPROM.write(0, 1);
    for (int i = 0; i < BASE_IMAGE_SIZE; ++i) EEPROM.write(i + 1, baseImage[i]);
    EEPROM.commit();
  } else {
    // Compare with base image
    int diffSum = 0;
    for (int i = 0; i < BASE_IMAGE_SIZE; ++i) {
      diffSum += abs((int)fb->buf[i] - (int)baseImage[i]);
    }
    int meanDiff = diffSum / BASE_IMAGE_SIZE;
    Serial.printf("Mean pixel diff: %d\n", meanDiff);
    if (meanDiff > CHANGE_THRESHOLD) {
      shouldSend = true;
      Serial.println("Change detected, sending image...");
      // Update base image
      memcpy(baseImage, fb->buf, BASE_IMAGE_SIZE);
      EEPROM.write(0, 1);
      for (int i = 0; i < BASE_IMAGE_SIZE; ++i) EEPROM.write(i + 1, baseImage[i]);
      EEPROM.commit();
    } else {
      Serial.println("No significant change.");
    }
  }

  if (shouldSend) {
    // Downsample and send via ESP-NOW
    downsample_160x120_to_80x60((uint8_t*)fb->buf, downsampledImage);
    pack_4bit(downsampledImage, packedImage, DOWNSAMPLED_IMAGE_SIZE);
    sendImageESPNow(packedImage, DOWNSAMPLED_PACKED_SIZE);
  }

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