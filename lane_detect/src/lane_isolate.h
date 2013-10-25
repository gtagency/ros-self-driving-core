#ifndef __LANE_ISOLATE_H
#define __LANE_ISOLATE_H

#include "cv.h"
#include "lane.h"

namespace lane_isolate {

double laneIsolate(cv::Mat& output, const cv::Mat& input);

}

#endif//__LANE_ISOLATE_H
