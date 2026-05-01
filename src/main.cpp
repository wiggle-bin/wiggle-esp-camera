#include <Arduino.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "env.h"
#include <Wire.h>
#include <BH1750.h>
#include <HTTPClient.h>
#include <esp_sleep.h>

// Send images
const char *serverBaseUrl = "http://homeassistant.local:8123/api/wiggle/upload";

// Utility: generate a timestamp string (optional)
String getTimestampedFilename()
{
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);
  char buffer[32];
  strftime(buffer, sizeof(buffer), "esp32_%Y%m%d_%H%M%S.jpg", timeinfo);
  return String(buffer);
}

// Send photo from RAM
void sendPhoto(camera_fb_t *fb)
{
  if (!fb || !fb->buf || fb->len == 0)
  {
    Serial.println("Invalid frame buffer");
    return;
  }

  String fileName = getTimestampedFilename();
  String serverUrl = String(serverBaseUrl);

  HTTPClient http;
  http.begin(serverUrl);
  http.addHeader("Content-Type", "image/jpeg");

  int httpResponseCode = http.POST(fb->buf, fb->len);
  if (httpResponseCode > 0)
  {
    Serial.printf("Photo sent successfully! Response: %d\n", httpResponseCode);
  }
  else
  {
    Serial.printf("Photo send failed: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
}

// MQTT client
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
  for (int i = 0; i < NUM_LEDS; i++)
  {
    strip.setPixelColor(i, 0);
  }
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
  sendPhoto(fb);
  
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

void setupLedFlash(int pin);

void setup_wifi()
{
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected.");
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

void setup()
{
  if (LIGHT_ALWAYS_ON)
  {
    // Soft white, low brightness (e.g., 64 out of 255)
    turnOnLedRing(255, 255, 255, 64);
  }

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // === Wifi and MQTT server ===
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

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
  if (SMALL_IMAGE_SIZE)
  {
    config.pixel_format = PIXFORMAT_JPEG; // Use JPEG for compatibility
    config.frame_size = FRAMESIZE_QVGA;   // 320x240, small but enough for activity
    config.jpeg_quality = 15;             // Lower quality for smaller size
  }
  else
  {
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_SVGA; // 800x600
    config.jpeg_quality = 8;            // Higher quality
  }
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

  // Initialize SD card
  if (!SD.begin(21))
  {
    Serial.println("Card Mount Failed");
  }
  uint8_t cardType = SD.cardType();

  // Determine if the type of SD card is available
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD card attached");
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }

  sd_sign = true; // sd initialization check passes

  // Temp
  sensors.begin();

  // Start I2C on custom pins
  Wire.begin(5, 6); // SDA = 5, SCL = 6

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
  {
    Serial.println("BH1750 sensor initialized");
  }
  else
  {
    Serial.println("Error initializing BH1750 sensor. Check wiring.");
  }
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }

  // Light sensor
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  delay(1000);

  // Temp sensors
  sensors.requestTemperatures();
  int deviceCount = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(deviceCount);
  Serial.println(" DS18B20 sensor(s).");

  for (int i = 0; i < deviceCount; i++)
  {
    float tempC = sensors.getTempCByIndex(i);
    // tempC += calibrationOffsets[i];

    DeviceAddress sensorAddress;
    sensors.getAddress(sensorAddress, i);

    char sensorID[17];
    addressToString(sensorAddress, sensorID);

    Serial.print("Sensor ID ");
    Serial.print(sensorID);
    Serial.print(": ");
    Serial.print(tempC);
    Serial.println(" °C");

    char tempString[8];
    dtostrf(tempC, 6, 2, tempString);

    String topic = "home/sensors/" + String(sensorID);
    bool success = client.publish(topic.c_str(), tempString);

    if (success)
    {
      Serial.println("MQTT message published successfully.");
    }
    else
    {
      Serial.println("Failed to publish MQTT message.");

      if (!client.connected())
      {
        Serial.println("MQTT client not connected!");
      }

      Serial.print("MQTT client state: ");
      Serial.println(client.state());
    }
  }

  // Servo removed

  // Camera & SD available, take a picture (if enabled)
  if (TAKE_PICTURES)
  {
    if (camera_sign)
    {
      char filename[32];
      sprintf(filename, "/image%d.jpg", imageCount);
      photo_save(filename, sd_sign);
      Serial.printf("Saved picture: %s\r\n", filename);
      imageCount++;
    }
    else
    {
      Serial.printf("camera_sign = %d", camera_sign);
    }
  }
  else
  {
    Serial.println("Picture taking is disabled (TAKE_PICTURES = false)");
  }

  if (DEEP_SLEEP)
  {
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

void takeAndPrintPhoto()
{
  camera_fb_t *fb = esp_camera_fb_get(); // Capture the photo
  if (!fb)
  {
    Serial.println("Camera capture failed");
    return;
  }

  Serial.printf("Captured photo %d bytes\n", fb->len);

  // Here, you can add any further processing if needed, like sending the photo over a network.
  // For now, just print the length of the photo.

  // Return the frame buffer back to the camera
  esp_camera_fb_return(fb);
}