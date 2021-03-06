#ifndef __GROUND_TRANSFORM_PROJECTIVE_H
#define __GROUND_TRANSFORM_PROJECTIVE_H

#include "ground_transform.h"

namespace ld {
    class GroundTransformProjective : public GroundTransform {

    public:
		void transform(const cv::Mat& src, cv::Mat& dest) const;
        void transform(const cv::Mat& src, cv::Mat& dest, double angle_tweak) const;
    };
}

#endif//__GROUND_TRANSFORM_PROJECTIVE_H
