
#include "lane_extract_cv.h"

using namespace ld;

LaneExtractCv::LaneExtractCv(const cv::Mat& src, const GroundTransform& gtrans) {
    doLaneExtraction(src, gtrans);
}

LaneExtractCv::~LaneExtractCv() {
}

int LaneExtractCv::numLanes() {
   return lanes.size();
}

void LaneExtractCv::describeLane(int num, Lane& lane) {
}


void LaneExtractCv::doLaneExtraction(const cv::Mat& src, const GroundTransform& gtrans) {
    cv::Mat dst;
    gtrans.transform(src, dst);

    //find lanes in resulting transform and fill in this->lanes 
}
