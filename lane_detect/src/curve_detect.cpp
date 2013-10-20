
#include "curve_detect.h"

#include "math.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

using namespace cd;
using namespace cv;

// #define DEBUG

CurveDetect::CurveDetect(int bh, int bw)
    : boxHeight(bh), boxWidth(bw) {
    A = (Mat_<float>(2,2) << 1, 0, 0, -1);
}

/*
const int channels = I.channels();
    switch(channels)
    {
    case 1:
        {
            for( int i = 0; i < I.rows; ++i)
                for( int j = 0; j < I.cols; ++j )
                    I.at<uchar>(i,j) = table[I.at<uchar>(i,j)];
            break;
        }
    case 3:
        {
         Mat_<Vec3b> _I = I;

         for( int i = 0; i < I.rows; ++i)
            for( int j = 0; j < I.cols; ++j )
               {
                   _I(i,j)[0] = table[_I(i,j)[0]];
                   _I(i,j)[1] = table[_I(i,j)[1]];
                   _I(i,j)[2] = table[_I(i,j)[2]];
            }
         I = _I;
         break;
        }
    }

    return I;
    */

double CurveDetect::findBestTheta(const Mat& roi, Point center, int radius) {
    double theta;
    double twoPi = (2 * M_PI);
    bool found = false;
    //find the first "theta" by sweeping the region of interest for the first non zero pixel
    for (int ii = 0; ii < 360; ii++) {
        int x = boxWidth/2 + radius * cos(ii / twoPi);
        int y = boxHeight/2 + radius * sin(ii / twoPi);
#ifdef DEBUG
        printf("%d", roi.at<uchar>(y,x));
#endif
        if (roi.at<uchar>(y,x) != 0) {
            theta = ii / twoPi;
            found = true;
            break;
        }
    }
    if (!found) {
        return -1;
    }
    return theta;
}

void CurveDetect::canny(const Mat& src, Mat& edges) {
    //TODO: may have to get clever about selecting threshold
    int edgeThresh = 1;
    double low_thres = 30; //120;
    int const max_lowThreshold = 200;
    int ratio = 3;
    int kernel_size = 3;
    cv::Mat detected_edges;
    
    Mat junk;
    // double high_thres = cv::threshold( src, junk, 0, 255, CV_THRESH_BINARY+CV_THRESH_OTSU ) ;
    // printf("high_thres: %f\n", high_thres);
    // lowThreshold = high_thres * 0.5;
    /// Reduce noise with a kernel 3x3
    double high_thres = low_thres*ratio;
    cv::blur( src, detected_edges, cv::Size(3,3) );
    /// Canny detector
    cv::Canny( detected_edges, detected_edges, low_thres, low_thres, kernel_size );

    /// Using Canny's output as a mask, we display our result
    edges = cv::Scalar::all(0);
#ifdef DEBUG
    printf("Detected Size: %d %d\n", detected_edges.rows, detected_edges.cols);
#endif
    src.copyTo( edges, detected_edges);
    // edges = src;
}

/**
 * Rotate an image
 */
void CurveDetect::rotate(const cv::Mat& src, Point center, double angleRad, cv::Mat& dst) {
#ifdef DEBUG
    printf("Rotate image: %f radians\n", angleRad);
#endif
    int len = std::max(src.cols, src.rows);
    cv::Mat r = cv::getRotationMatrix2D(center, angleRad * 180.0/M_PI, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len));
}

Rect CurveDetect::getROIRect(const Point& pt) {
    //TODO: may have to be negative or something
    float x = fmax(0, pt.x - boxWidth/2);
    float y = fmax(0, pt.y - boxHeight);
#ifdef DEBUG
    printf("ROI Rect: %f, %f, %d, %d\n", x, y, boxWidth, boxHeight);
#endif
    return Rect(x, y, boxWidth, boxHeight);
}

Rect CurveDetect::getTileRect(const Point& pt) {
    //TODO: may have to be negative or something
    float x = fmax(0, pt.x - boxWidth * 2);
    float y = fmax(0, pt.y - boxHeight * 4);
    printf("%f, %f, %d, %d\n", x, y, boxWidth * 4, boxHeight * 4);
    return Rect(x, y, boxWidth * 4, boxHeight * 4);
}

//points is a n x 2 matrix, with x in first column and y in the second column
void CurveDetect::getPointsOfInterest(const Mat& roi, Mat& points) {
#ifdef DEBUG
    printf("Get points of interest\n");
#endif
    Mat edges;
    canny(roi, edges);
    Mat locations;   // output, locations of non-zero pixels 
    //locations should be a n x 1 vector of points
    findNonZero(edges, locations);
#ifdef DBEUG
    printf("Found edges: %d, %d\n", locations.rows, locations.cols);
    debugShowImage(edges);
#endif
    //adjust the points so they're relative to axes at the bottom center of the box.
    points = Mat::zeros(locations.rows, 2, CV_32FC1);
    for (int r = 0; r < locations.rows; r++) {
        Point *locPtr = locations.ptr<Point>(r);
        float *ptPtr  = points.ptr<float>(r);
#ifdef DEBUG
        printf("%d, %d\n", locPtr[0].x, locPtr[0].y);
#endif
        ptPtr[0] = locPtr[0].x;
        ptPtr[1] = locPtr[0].y;
    } 
}

Mat CurveDetect::polypoint(const Mat& poly, float x) {
    //TODO: currently only works with 1 degree polynomial...can expand to n degree using varargs
#ifdef DEBUG
    printf("Poly: %f, %f, x: %f\n", poly.at<float>(0), poly.at<float>(1), x);
#endif 
    return (Mat_<float>(2,1) << x, poly.at<float>(0) + x * poly.at<float>(1));
}

Mat CurveDetect::getBestPointInImage(const Mat& bottom, const Mat& top, const Mat& imageStart, double rotation) {
    Mat Rt = (Mat_<float>(2,2) << cos(rotation), -sin(rotation),
                                  sin(rotation),  cos(rotation));
    Rt = Rt.t();
                              
    Mat adjTop = A * Rt * A * top + imageStart;
    Mat adjBot = A * Rt * A * bottom + imageStart;
#ifdef DEBUG
    printf("%f, %f\n", adjTop.at<float>(0), adjTop.at<float>(1));
    printf("%f, %f\n", adjBot.at<float>(0), adjBot.at<float>(1));
#endif
    Mat nextPt = adjTop;
    if (norm(imageStart - adjTop) < norm(imageStart - adjBot)) {
        nextPt = adjBot;
    }
#ifdef DEBUG
    printf("%f, %f\n", imageStart.at<float>(0), imageStart.at<float>(1));
    printf("%f, %f\n", nextPt.at<float>(0), nextPt.at<float>(1));
#endif
    return nextPt;
}

void CurveDetect::fitPoly(const Mat& points, Mat& bottom, Mat& top, float& lineAngle) {
    //fit a y,x polynomial, so we can compute x values for the top and bottom
    // of the box
    Mat xsMat = points.col(0) - boxWidth/2;
    Mat ysMat = points.col(1) - boxHeight;
    Mat poly;
    poly = Mat::zeros(2, 1,CV_32FC1);
#ifdef DEBUG
    printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", xsMat.rows, xsMat.cols, xsMat.type(), ysMat.rows, ysMat.cols, ysMat.type(), poly.rows, poly.cols, poly.type());
    for (int r = 0; r < xsMat.rows; r++) {
        printf("%f, %f\n", xsMat.ptr<float>(r)[0], ysMat.ptr<float>(r)[0]);
    } 
#endif
    //this returns a polynomial in poly, in the right order
    // but remember, this is a y,x polynomial, so poly(0) is x intercept
    polyfit(ysMat, xsMat, poly, 1);

    //these points are x,y, due to call to flip
    top = polypoint(poly, -boxHeight);
    flip(top, top, -1);

    bottom = polypoint(poly, 0);
    flip(bottom, bottom, -1);

    lineAngle = tan(poly.at<float>(1));
#ifdef DEBUG
    printf("Polynomial: %f, %f\n", poly.at<float>(0), poly.at<float>(1));
    
    printf("DEBUG testing the poly points\n");
    printf("Top of roi: %f, %f\n", top.at<float>(0), top.at<float>(1));
    printf("Bottom of roi: %f, %f\n", bottom.at<float>(0), bottom.at<float>(1));
#endif
}

void CurveDetect::debugShowImage(const Mat& img) {
#ifdef DEBUG
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", img );                   // Show our image inside it.
    waitKey(0);
#endif
}

//NOTE: assumes src is a binary image
void CurveDetect::fitCurve(const Mat& src, const Point& startPt, int maxCurves) {
    int curves = 0;
    Size size = src.size();
#ifdef DEBUG
    printf("Source Size: %d %d\n", src.rows, src.cols);
    //TODO: preprocess image? convert color, etc
    printf("Detecting best initial theta\n");
#endif
    Mat roi = Mat(src, getROIRect(startPt));
    //NOTE: this assumes that the curve is at least as big as one box in width and length
    // also assumes width, height >= 2
    float theta0 = M_PI/2.7;//findBestTheta(roi, Point(boxWidth/2, boxHeight/2), fmin(boxWidth/2, boxHeight/2));

#ifdef DEBUG
    printf("Best theta: %.02f", theta0);
    printf("Start curve fitting\n");
#endif
    //clear the current vectors
    points.clear();
    thetas.clear();

    points.push_back(startPt);
    thetas.push_back(theta0);

    Point pt = startPt;

#ifdef DEBUG    
    Mat show = src.clone();
#endif
    
    float theta = theta0;
    while (pt.x < size.width && pt.y < size.height && curves < maxCurves) {
        float rotation = M_PI/2 - theta;
        //get the region of interest...the region we're looking at
        // Mat tile = Mat(src, getTileRect(pt));
        // debugShowImage(tile);
        Mat rot;
        // rotate(tile, Point(boxWidth, boxHeight), rotation, rot);
        rotate(src, pt, rotation, rot);
        debugShowImage(rot);

        // roi = Mat(rot, getROIRect(Point(tile.size().width - boxWidth/4, tile.size().height - boxHeight / 2)));
        roi = Mat(rot, getROIRect(pt));
        debugShowImage(roi);

        Mat poi;
        //get points of interest to consider when fitting the line
        getPointsOfInterest(roi, poi);

        float lineAngle;
        Mat bottom, top;
        //fit a line to these points, and compute the points on this line
        // at the bottom and top of the region of interest 
        fitPoly(poi, bottom, top, lineAngle);
        
        Mat ptMat = (Mat_<float>(2,1) << pt.x, pt.y);

        //compute
        Mat nextPtMat = getBestPointInImage(bottom, top, ptMat, rotation);
        Point nextPt  = Point(nextPtMat.at<float>(0), nextPtMat.at<float>(1));

#ifdef DEBUG
        line(show, pt, nextPt, Scalar(127, 127, 127));
        debugShowImage(show);
        printf("%f, %f\n", lineAngle, theta);
#endif
        pt = nextPt;
        theta = lineAngle + theta;

        points.push_back(pt);
        thetas.push_back(theta);
        curves++;
    }
    
//     points = zeros(2,4);
//     thetas = zeros(1,4);
//     p = p0;
//     theta = theta0;
// 
//     for i=1:num
//         points(:,i) = p;
//         thetas(i) = theta;
//         [p theta] = aoi_func(img, p, theta, aoi_width, aoi_height);
//     end
//     points(:,i+1) = p;
//     thetas(i+1) = theta;
}

std::vector<Point>& CurveDetect::getPointsOnCurve() {
    return this->points;
}
std::vector<float>& CurveDetect::getAnglesOfIncidence() {
    return this->thetas;
}
