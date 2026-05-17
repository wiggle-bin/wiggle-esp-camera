#include <Arduino.h>
#include "env.h"
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include "camera_capture.h"
#include "image_transmission.h"

ImageTransmissionState imageTransmissionState;
ImageTransmissionConfig imageTransmissionConfig = {RECEIVER_MAC, 9, 0};

// Pin connected to IRLB8747PBF gate (GPIO3) to turn off power to LED ring via MOSFET
#define MOSFET_PIN 3

// LED ring
#define LED_PIN 44 // Adjust if using a different GPIO
#define NUM_LEDS 8

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// === Feature Toggles ===
#define READ_SENSORS_MINUTES 0.2      // Time in minutes between sensor readings
#define READ_SENSORS_INTERVAL (READ_SENSORS_MINUTES * 60UL * 1000UL) // Time in ms

// === Energy saving Toggles ===
#define DEEP_SLEEP true             // Set to true deep sleep between sensor readings and photo captures
#define DEEP_SLEEP_MINUTES 0.2       // Deep sleep duration in minutes
#define DEEP_SLEEP_INTERVAL_US (DEEP_SLEEP_MINUTES * 60ULL * 1000000ULL)

WiFiClient espClient;

// Timing variables
unsigned long setupStartTime = 0;
unsigned long setupEndTime = 0;
unsigned long loopStartTime = 0;
unsigned long loopEndTime = 0;

void turnOnLedRing(uint8_t r = 255, uint8_t g = 0, uint8_t b = 0, uint8_t brightness = 60)
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

void setup() {  
  setupStartTime = millis();

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  initializeBaseImagePersistence(imageTransmissionState);

  // Turn on power to LED ring via NPN mosfet
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);

  // === ESP-NOW ===
  initializeESPNowSender(imageTransmissionConfig);

  // Turning on LED in setup, because right before camera requires a delay in between to illuminate the scene
  // To save energy the LED ring is turned on in setup and camera setup is the delay needed for illumination
  turnOnLedRing(); // Default: red at brightness 255

  if (!initializeCamera()) {
    return;
  }

  setupEndTime = millis();
}

void loop() {
  loopStartTime = millis();

  constexpr bool USE_DUMMY_READ = true;
  camera_fb_t *fb = capturePhotoFrame(DEEP_SLEEP,
                                      USE_DUMMY_READ,
                                      IMAGE_TRANSMISSION_BASE_IMAGE_SIZE);
  
  // Turn off LED ring after capture to save energy
  turnOffLedRing();
  // Turn LED ring power off via MOSFET to save energy during deep sleep
  digitalWrite(MOSFET_PIN, LOW);  // MOSFET OFF
  pinMode(LED_PIN, INPUT); // High-Z prevents parasitic powering through data line

  if (!fb) {
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
    Serial.printf("Waiting %.2f minutes...\n", (double)READ_SENSORS_INTERVAL / 60000.0);
    delay(READ_SENSORS_INTERVAL);

    // Immitating setup delay
    delay(42); // Image persistantce delay + esp-now setup delay
    // Turn on power to LED ring via NPN mosfet
    pinMode(MOSFET_PIN, OUTPUT);
    digitalWrite(MOSFET_PIN, HIGH); 
    // Turn on LED ring
    turnOnLedRing();
    delay(388); // Camera setup
  }
}