#pragma once

#include <cstdint>
#include "cereal/visionipc/visionipc.h"

class VideoEncoder {
public:
  virtual ~VideoEncoder() {}
  virtual int encode_frame(const uint8_t *y_ptr, const uint8_t *u_ptr, const uint8_t *v_ptr,
                           int in_width, int in_height, VisionIpcBufExtra *extra) = 0;
  virtual void encoder_open(const char* path) = 0;
  virtual void encoder_close() = 0;
};
