
#include "cv_bridge_ex.h"
#include "cv.h"
#include <sstream>

#include <sensor_msgs/image_encodings.h>
namespace enc = sensor_msgs::image_encodings;

const std::string& getRosType(int cvType) {
    switch(cvType) {
    case CV_8UC3: return enc::BGR8;
    case CV_8UC1: return enc::MONO8;
    default:
        std::stringstream s;

        s << cvType;

        throw std::runtime_error("Unrecognized Opencv type [" + s.str() + "]");
    }
/*
// Check for the most common encodings first
  if (encoding == enc::BGR8)   return CV_8UC3;
  if (encoding == enc::MONO8)  return CV_8UC1;
  if (encoding == enc::RGB8)   return CV_8UC3;
  if (encoding == enc::MONO16) return CV_16UC1;
  if (encoding == enc::BGR16)  return CV_16UC3;
  if (encoding == enc::RGB16)  return CV_16UC3;
  if (encoding == enc::BGRA8)  return CV_8UC4;
  if (encoding == enc::RGBA8)  return CV_8UC4;
  if (encoding == enc::BGRA16) return CV_16UC4;
  if (encoding == enc::RGBA16) return CV_16UC4;

  // For bayer, return one-channel
  if (encoding == enc::BAYER_RGGB8) return CV_8UC1;
  if (encoding == enc::BAYER_BGGR8) return CV_8UC1;
  if (encoding == enc::BAYER_GBRG8) return CV_8UC1;
  if (encoding == enc::BAYER_GRBG8) return CV_8UC1;
  if (encoding == enc::BAYER_RGGB16) return CV_16UC1;
  if (encoding == enc::BAYER_BGGR16) return CV_16UC1;
  if (encoding == enc::BAYER_GBRG16) return CV_16UC1;
  if (encoding == enc::BAYER_GRBG16) return CV_16UC1;

  // Miscellaneous
  if (encoding == enc::YUV422) return CV_8UC2;

  // Check all the generic content encodings
#define CHECK_ENCODING(code)                            \
  if (encoding == enc::TYPE_##code) return CV_##code    \
//
#define CHECK_CHANNEL_TYPE(t)                   \
  CHECK_ENCODING(t##1);                         \
  CHECK_ENCODING(t##2);                         \
  CHECK_ENCODING(t##3);                         \
  CHECK_ENCODING(t##4);                         \
//

  CHECK_CHANNEL_TYPE(8UC);
  CHECK_CHANNEL_TYPE(8SC);
  CHECK_CHANNEL_TYPE(16UC);
  CHECK_CHANNEL_TYPE(16SC);
  CHECK_CHANNEL_TYPE(32SC);
  CHECK_CHANNEL_TYPE(32FC);
  CHECK_CHANNEL_TYPE(64FC);

#undef CHECK_CHANNEL_TYPE
#undef CHECK_ENCODING

  throw Exception("Unrecognized image encoding [" + encoding + "]");
}*/
}
