#ifndef __GROUND_TRANSFORM_H
#define __GROUND_TRANSFORM_H

#include "cv.h"

namespace ld {
    class GroundTransform {
    
    public:
        virtual void transform(const cv::Mat& src, cv::Mat& dest) const = 0;
    };
}
#endif //__GROUND_TRANSFORM_H
