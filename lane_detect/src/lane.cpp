
#include "lane.h"

using namespace ld;

const std::vector<cv::Point>& Lane::getPoints() {
    return this->points;
}
