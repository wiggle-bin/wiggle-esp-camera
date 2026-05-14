#include "image_transmission.h"

#include <Arduino.h>
#include <esp_now.h>

extern uint8_t peerAddress[];

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

void pack_4bit(const uint8_t *src, uint8_t *dst, int num_pixels) {
  for (int i = 0; i < num_pixels / 2; ++i) {
    uint8_t hi = quantize_4bit(src[i * 2]);
    uint8_t lo = quantize_4bit(src[i * 2 + 1]);
    dst[i] = (hi << 4) | (lo & 0x0F);
  }
}

void sendImageESPNow(const uint8_t *packed, size_t packed_len) {
  const size_t chunkSize = 200;
  uint16_t total_chunks = (packed_len + chunkSize - 1) / chunkSize;

  for (uint16_t chunk = 0; chunk < total_chunks; ++chunk) {
    size_t offset = chunk * chunkSize;
    size_t len = (offset + chunkSize > packed_len) ? (packed_len - offset) : chunkSize;
    uint8_t buf[chunkSize + 4];

    buf[0] = static_cast<uint8_t>(chunk & 0xFF);
    buf[1] = static_cast<uint8_t>((chunk >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>(total_chunks & 0xFF);
    buf[3] = static_cast<uint8_t>((total_chunks >> 8) & 0xFF);
    memcpy(buf + 4, packed + offset, len);

    esp_err_t result = esp_now_send(peerAddress, buf, len + 4);
    if (result == ESP_OK) {
      Serial.printf("ESP-NOW chunk %d/%d sent\n", chunk + 1, total_chunks);
    } else {
      Serial.printf("ESP-NOW send failed: %d\n", result);
    }
    delay(10);
  }
}