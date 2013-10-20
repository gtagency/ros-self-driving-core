#ifndef __LANE_H
#define __LANE_H

#include <vector>
#include <cv.h>

namespace ld {
    class Lane {
    private:
        int type;
        std::vector<cv::Point> points;
    public:
        const std::vector<cv::Point>& getPoints();

    };
}
#endif

