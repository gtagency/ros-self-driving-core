#ifndef __GROUND_TRANSFORM_SPHERE_H
#define __GROUND_TRANSFORM_SPHERE_H

#include "ground_transform.h"

namespace ld {
    class GroundTransformSphere : public GroundTransform {
    
    public:
        void transform(const cv::Mat& src, cv::Mat& dest) const;
    };
}
#endif //__GROUND_TRANSFORM_SPHERE_H
