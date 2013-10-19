#ifndef __LANE_H
#define __LANE_H

#include <vector>
#include "point.h"

namespace ld {
    class Lane {
    private:
        int type;
        std::vector<Point> points;
    public:
        const std::vector<Point>& getPoints();

    };
}
#endif

