
#include "lane.h"

using namespace ld;

Lane::Lane(const std::vector<cv::Point>& points) {
   this->points.insert(this->points.end(), points.begin(), points.end()); 
}

const std::vector<cv::Point>& Lane::getPoints() const {
    return this->points;
}
