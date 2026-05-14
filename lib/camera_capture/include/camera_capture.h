#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <stddef.h>
#include <stdint.h>

#include "esp_camera.h"

// Initializes camera with the project defaults for XIAO ESP32S3.
// Returns true on success.
bool initializeCamera();

// Captures one frame and validates its expected byte size.
// Returns nullptr when capture fails or frame size does not match.
camera_fb_t *capturePhotoFrame(bool deepSleepEnabled, bool useDummyRead, size_t expectedFrameSize);

#endif