
#ifndef __LANE_EXTRACT_CV_H
#define __LANE_EXTRACT_CV_H

#include "cv.h"
#include "lane.h"
#include "ground_transform.h"

namespace ld {
    class LaneExtractCv {
    private:
        std::vector<Lane> lanes;

        void doLaneExtraction(const cv::Mat& src, const GroundTransform& gtrans);

    public:
        LaneExtractCv(const cv::Mat& src, const GroundTransform& gtrans);
        ~LaneExtractCv();

        int numLanes();

        void describeLane(int num, Lane& lane);
    };
}

#endif //__LANE_EXTRACT_CV_H
