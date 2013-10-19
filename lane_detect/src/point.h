
#ifndef __POINT_H
#define __POINT_H

namespace ld {
    class Point {

    private:
        int x;
        int y;

    public:
        Point(int x, int y);

        int getX() const;
        int getY() const;
    };
}

#endif
