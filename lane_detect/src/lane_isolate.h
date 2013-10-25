#ifndef __LANE_ISOLATE_H
#define __LANE_ISOLATE_H

#include "cv.h"
#include "lane.h"

namespace lane_isolate {

double laneIsolate(const cv::Mat& input, cv::Mat& output, std::vector<std::vector<cv::Point> >& polygons);

}

#endif//__LANE_ISOLATE_H
