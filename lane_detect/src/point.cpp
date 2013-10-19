
#include "point.h"

using namespace ld;

Point::Point(int x, int y) {
    this->x = x;
    this->y = y;
}

int Point::getX() const {
    return this->x;
}
int Point::getY() const {
    return this->y;
}
