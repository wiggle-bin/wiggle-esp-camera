#include "image_transmission.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

bool initializeESPNowSender(const ImageTransmissionConfig &config) {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(config.wifiChannel, WIFI_SECOND_CHAN_NONE);
  Serial.print("ESP-NOW sender using channel: ");
  Serial.println(config.wifiChannel);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return false;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, config.peerAddress, sizeof(config.peerAddress));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(config.peerAddress)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add ESP-NOW peer");
      return false;
    }
  }

  return true;
}

void initializeBaseImagePersistence(ImageTransmissionState &state) {
  EEPROM.begin(IMAGE_TRANSMISSION_BASE_IMAGE_SIZE + 1);
  state.baseImageValid = EEPROM.read(0) == 1;
  if (!state.baseImageValid) {
    return;
  }

  for (size_t i = 0; i < IMAGE_TRANSMISSION_BASE_IMAGE_SIZE; ++i) {
    state.baseImage[i] = EEPROM.read(i + 1);
  }
}

ImageMetadata processFrameAndSendIfChanged(const uint8_t *frame,
                                           ImageTransmissionState &state,
                                           const ImageTransmissionConfig &config) {
  // Always compute average grey value of the incoming frame
  uint32_t greySum = 0;
  for (size_t i = 0; i < IMAGE_TRANSMISSION_BASE_IMAGE_SIZE; ++i) {
    greySum += frame[i];
  }
  uint8_t avgGrey = static_cast<uint8_t>(greySum / IMAGE_TRANSMISSION_BASE_IMAGE_SIZE);

  ImageMetadata metadata = {0, 0, 0, avgGrey, false};

  if (!state.baseImageValid) {
    memcpy(state.baseImage, frame, IMAGE_TRANSMISSION_BASE_IMAGE_SIZE);
    state.baseImageValid = true;
    Serial.println("Base image captured.");
    EEPROM.write(0, 1);
    for (size_t i = 0; i < IMAGE_TRANSMISSION_BASE_IMAGE_SIZE; ++i) {
      EEPROM.write(i + 1, state.baseImage[i]);
    }
    EEPROM.commit();
    sendMetadataESPNow(config.peerAddress, metadata);
    return metadata;
  }

  int diffSum = 0;
  uint8_t maxDiff = 0;
  uint16_t changedPixelCount = 0;
  for (size_t i = 0; i < IMAGE_TRANSMISSION_BASE_IMAGE_SIZE; ++i) {
    uint8_t d = static_cast<uint8_t>(abs((int)frame[i] - (int)state.baseImage[i]));
    diffSum += d;
    if (d > maxDiff) maxDiff = d;
    if (d > config.changeThreshold) ++changedPixelCount;
  }

  uint16_t meanDiff = static_cast<uint16_t>(diffSum / IMAGE_TRANSMISSION_BASE_IMAGE_SIZE);
  metadata.meanDiff = meanDiff;
  metadata.maxDiff = maxDiff;
  metadata.changedPixelCount = changedPixelCount;
  Serial.printf("Mean pixel diff: %d, max: %d, changed pixels: %d\n", meanDiff, maxDiff, changedPixelCount);

  if (meanDiff > config.changeThreshold) {
    Serial.println("Change detected, sending image...");
    memcpy(state.baseImage, frame, IMAGE_TRANSMISSION_BASE_IMAGE_SIZE);
    EEPROM.write(0, 1);
    for (size_t i = 0; i < IMAGE_TRANSMISSION_BASE_IMAGE_SIZE; ++i) {
      EEPROM.write(i + 1, state.baseImage[i]);
    }
    EEPROM.commit();

    downsample_160x120_to_80x60(frame, state.downsampledImage);
    pack_4bit(state.downsampledImage,
              state.packedImage,
              IMAGE_TRANSMISSION_DOWNSAMPLED_IMAGE_SIZE);
    sendImageESPNow(config.peerAddress,
                    state.packedImage,
                    IMAGE_TRANSMISSION_DOWNSAMPLED_PACKED_SIZE);
    metadata.imageSent = true;
  } else {
    Serial.println("No significant change.");
  }

  sendMetadataESPNow(config.peerAddress, metadata);
  return metadata;
}

void downsample_160x120_to_80x60(const uint8_t *src, uint8_t *dst) {
  for (int y = 0; y < IMAGE_TRANSMISSION_DOWNSAMPLED_HEIGHT; ++y) {
    for (int x = 0; x < IMAGE_TRANSMISSION_DOWNSAMPLED_WIDTH; ++x) {
      int sum = 0;
      int src_x = x * 2;
      int src_y = y * 2;
      sum += src[(src_y + 0) * 160 + (src_x + 0)];
      sum += src[(src_y + 0) * 160 + (src_x + 1)];
      sum += src[(src_y + 1) * 160 + (src_x + 0)];
      sum += src[(src_y + 1) * 160 + (src_x + 1)];
      dst[y * IMAGE_TRANSMISSION_DOWNSAMPLED_WIDTH + x] = sum / 4;
    }
  }
}

void pack_4bit(const uint8_t *src, uint8_t *dst, int num_pixels) {
  for (int i = 0; i < num_pixels / 2; ++i) {
    uint8_t hi = quantize_4bit(src[i * 2]);
    uint8_t lo = quantize_4bit(src[i * 2 + 1]);
    dst[i] = (hi << 4) | (lo & 0x0F);
  }
}

void sendImageESPNow(const uint8_t *peerAddress, const uint8_t *packed, size_t packed_len) {
  const size_t chunkSize = 200;
  uint16_t total_chunks = (packed_len + chunkSize - 1) / chunkSize;

  for (uint16_t chunk = 0; chunk < total_chunks; ++chunk) {
    size_t offset = chunk * chunkSize;
    size_t len = (offset + chunkSize > packed_len) ? (packed_len - offset) : chunkSize;
    uint8_t buf[chunkSize + 5];

    buf[0] = IMAGE_PACKET_TYPE_IMAGE;
    buf[1] = static_cast<uint8_t>(chunk & 0xFF);
    buf[2] = static_cast<uint8_t>((chunk >> 8) & 0xFF);
    buf[3] = static_cast<uint8_t>(total_chunks & 0xFF);
    buf[4] = static_cast<uint8_t>((total_chunks >> 8) & 0xFF);
    memcpy(buf + 5, packed + offset, len);

    esp_err_t result = esp_now_send(peerAddress, buf, len + 5);
    if (result == ESP_OK) {
      Serial.printf("ESP-NOW chunk %d/%d sent\n", chunk + 1, total_chunks);
    } else {
      Serial.printf("ESP-NOW send failed: %d\n", result);
    }
    delay(10);
  }
}

void sendMetadataESPNow(const uint8_t *peerAddress, const ImageMetadata &metadata) {
  uint8_t buf[8];
  buf[0] = IMAGE_PACKET_TYPE_METADATA;
  buf[1] = static_cast<uint8_t>(metadata.meanDiff & 0xFF);
  buf[2] = static_cast<uint8_t>((metadata.meanDiff >> 8) & 0xFF);
  buf[3] = metadata.maxDiff;
  buf[4] = static_cast<uint8_t>(metadata.changedPixelCount & 0xFF);
  buf[5] = static_cast<uint8_t>((metadata.changedPixelCount >> 8) & 0xFF);
  buf[6] = metadata.avgGrey;
  buf[7] = metadata.imageSent ? 1 : 0;
  esp_err_t result = esp_now_send(peerAddress, buf, sizeof(buf));
  if (result == ESP_OK) {
    Serial.printf("Metadata sent — meanDiff: %d, maxDiff: %d, changedPixels: %d, avgGrey: %d, imageSent: %d\n",
                  metadata.meanDiff, metadata.maxDiff, metadata.changedPixelCount,
                  metadata.avgGrey, metadata.imageSent ? 1 : 0);
  } else {
    Serial.printf("Metadata send failed: %d\n", result);
  }
}