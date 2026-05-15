#ifndef IMAGE_TRANSMISSION_H
#define IMAGE_TRANSMISSION_H

#include <stddef.h>
#include <stdint.h>

// ESP-NOW packet type bytes
constexpr uint8_t IMAGE_PACKET_TYPE_IMAGE    = 0x01;
constexpr uint8_t IMAGE_PACKET_TYPE_METADATA = 0x02;

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

struct ImageMetadata {
  uint16_t meanDiff;          // Mean absolute pixel difference from base image
  uint8_t  maxDiff;           // Single highest pixel difference from base image
  uint16_t changedPixelCount; // Number of pixels where |diff| > changeThreshold
  uint8_t  avgGrey;           // Average grey value of the captured frame
  bool     imageSent;         // Whether the image was actually transmitted
};

bool initializeESPNowSender(const ImageTransmissionConfig &config);

void initializeBaseImagePersistence(ImageTransmissionState &state);

ImageMetadata processFrameAndSendIfChanged(const uint8_t *frame,
                                           ImageTransmissionState &state,
                                           const ImageTransmissionConfig &config);

void downsample_160x120_to_80x60(const uint8_t *src, uint8_t *dst);

inline uint8_t quantize_4bit(uint8_t v) {
  return v >> 4;
}

void pack_4bit(const uint8_t *src, uint8_t *dst, int num_pixels);

void sendImageESPNow(const uint8_t *peerAddress, const uint8_t *packed, size_t packed_len);

void sendMetadataESPNow(const uint8_t *peerAddress, const ImageMetadata &metadata);

#endif