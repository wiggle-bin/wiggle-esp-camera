#ifndef IMAGE_TRANSMISSION_H
#define IMAGE_TRANSMISSION_H

#include <stddef.h>
#include <stdint.h>

#define DOWNSAMPLED_WIDTH 80
#define DOWNSAMPLED_HEIGHT 60
#define DOWNSAMPLED_IMAGE_SIZE (DOWNSAMPLED_WIDTH * DOWNSAMPLED_HEIGHT)

void downsample_160x120_to_80x60(const uint8_t *src, uint8_t *dst);

inline uint8_t quantize_4bit(uint8_t v) {
  return v >> 4;
}

void pack_4bit(const uint8_t *src, uint8_t *dst, int num_pixels);

void sendImageESPNow(const uint8_t *packed, size_t packed_len);

#endif