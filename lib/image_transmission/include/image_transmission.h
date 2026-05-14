#ifndef IMAGE_TRANSMISSION_H
#define IMAGE_TRANSMISSION_H

#include <stddef.h>
#include <stdint.h>

constexpr size_t IMAGE_TRANSMISSION_BASE_IMAGE_SIZE = 160 * 120;
constexpr size_t IMAGE_TRANSMISSION_DOWNSAMPLED_WIDTH = 80;
constexpr size_t IMAGE_TRANSMISSION_DOWNSAMPLED_HEIGHT = 60;
constexpr size_t IMAGE_TRANSMISSION_DOWNSAMPLED_IMAGE_SIZE =
    IMAGE_TRANSMISSION_DOWNSAMPLED_WIDTH * IMAGE_TRANSMISSION_DOWNSAMPLED_HEIGHT;
constexpr size_t IMAGE_TRANSMISSION_DOWNSAMPLED_PACKED_SIZE =
    IMAGE_TRANSMISSION_DOWNSAMPLED_IMAGE_SIZE / 2;

struct ImageTransmissionState {
  uint8_t baseImage[IMAGE_TRANSMISSION_BASE_IMAGE_SIZE];
  bool baseImageValid = false;
  uint8_t downsampledImage[IMAGE_TRANSMISSION_DOWNSAMPLED_IMAGE_SIZE];
  uint8_t packedImage[IMAGE_TRANSMISSION_DOWNSAMPLED_PACKED_SIZE];
};

struct ImageTransmissionConfig {
  uint8_t peerAddress[6];
  uint8_t wifiChannel;
  int changeThreshold;
};

bool initializeESPNowSender(const ImageTransmissionConfig &config);

void initializeBaseImagePersistence(ImageTransmissionState &state);

void processFrameAndSendIfChanged(const uint8_t *frame,
                                  ImageTransmissionState &state,
                                  const ImageTransmissionConfig &config);

void downsample_160x120_to_80x60(const uint8_t *src, uint8_t *dst);

inline uint8_t quantize_4bit(uint8_t v) {
  return v >> 4;
}

void pack_4bit(const uint8_t *src, uint8_t *dst, int num_pixels);

void sendImageESPNow(const uint8_t *peerAddress, const uint8_t *packed, size_t packed_len);

#endif