#ifndef __CURVE_DETECT_H
#define __CURVE_DETECT_H

#include <cv.h>

using namespace cv;

namespace cd {
    class CurveDetect {
    private:

        std::vector<Point> points;
        std::vector<float> thetas;
        Mat A;

        int boxWidth, boxHeight;
        
        double findBestTheta(const Mat& roi, Point center, int radius);
        void canny(const Mat& src, Mat& edges);
        void rotate(const cv::Mat& src, Point center, double angleRad, cv::Mat& dst);
        Rect getROIRect(const Point& pt);
        Rect getTileRect(const Point& pt);
        void getPointsOfInterest(const Mat& roi, Mat& points);
        Mat polypoint(const Mat& poly, float x);
        
        void fitPoly(const Mat& points, Mat& bottom, Mat& top, float& lineAngle);
        Mat getBestPointInImage(const Mat& bottom, const Mat& top, const Mat& imageStart, double rotation);
        
        void debugShowImage(const Mat& img);
        
    public:
        CurveDetect(int boxHeight, int boxWidth);

        void fitCurve(const Mat& src, const Point& startPt, int maxCurves);
        std::vector<Point>& getPointsOnCurve();
        std::vector<float>& getAnglesOfIncidence();
        
    };
}
#endif //__CURVE_DETECT_H