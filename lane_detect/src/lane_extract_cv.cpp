
#include <vector>

#include "ground_transform_projective.h"
#include "lane_isolate.h"
#include "lane_extract_cv.h"
#include "curve_detect.h"
#include "math.h"

using namespace lane_isolate;
using namespace ld;
using namespace cd;
using namespace std;

LaneExtractCv::LaneExtractCv(const cv::Mat& src, const GroundTransform& gtrans, int maxLanes) {
    doLaneExtraction(src, gtrans, maxLanes);
}

LaneExtractCv::~LaneExtractCv() {
}

const std::vector<Lane>& LaneExtractCv::getLanes() const {
    return lanes;
}

const cv::Mat& LaneExtractCv::getProcessedImage() {
    return this->processed; 
}

int LaneExtractCv::getProcessedImageEnc() {
    return this->processed.type();
}

cv::Scalar LOW_HSV_EDGE = cv::Scalar(20, 0, 0);
cv::Scalar HIGH_HSV_EDGE = cv::Scalar(40, 255, 255);

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void cannyThreshold(const cv::Mat& src, const cv::Mat& srcGray, cv::Mat& dst, cv::Mat& dstMask) {
    int edgeThresh = 1;
    int lowThreshold = 60; //120;
    int const max_lowThreshold = 200;
    int ratio = 3;
    int kernel_size = 3;
    cv::Mat detected_edges;
    /// Reduce noise with a kernel 3x3
    cv::blur( srcGray, detected_edges, cv::Size(3,3) );
    /// Canny detector
    cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0);

    src.copyTo( dst, detected_edges);
    // getBinary(dst, LOW_HSV_EDGE, HIGH_HSV_EDGE, dstMask);
    // imshow( window_name, dst );
    cv::cvtColor(dst, dstMask, CV_BGR2GRAY);
      
}
/*
void flipHorizAndVert(Mat& img) {
	int flipMode = -1;
	IplImage ipl = img;
	cvFlip(&ipl, &ipl, flipMode);
}*/
/*
void pullFrame(VideoCapture& cap, Mat& img, Mat& imgGray, void (*adjustFunc)(Mat& img)) {
	Mat frame;
    cap >> frame; // get a new frame from camera
	img = frame;

	if (adjustFunc) {
		adjustFunc(img);
	}
    imgGray = img.clone();
    cvtColor(imgGray, imgGray, CV_BGR2GRAY);
}
*/
// Hardcoded values for green android folder
// Scalar LOW_HSV = Scalar(60, 50, 50);
// Scalar HIGH_HSV = Scalar(90, 255, 255);
// Yellow line
cv::Scalar LOW_HSV = cv::Scalar(25, 120, 160);
cv::Scalar HIGH_HSV = cv::Scalar(35, 255, 255);
// Scalar LOW_HSV = Scalar(20, 200, 250);
// Scalar HIGH_HSV = Scalar(40, 255, 255);

void removeSmall(cv::Mat& src, cv::Mat& dest, double minRadius) {
    vector<vector<cv::Point> > contours;
    cv::findContours(src.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    vector<vector<cv::Point> > erase;

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<cv::Point> > contours_poly( contours.size() );
    // vector<Rect> boundRect( contours.size() );
    // vector<Point2f>center( contours.size() );
    // vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ) {
        cv::Point2f center;
        float radius;
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
        // boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center, radius );
        if (radius < minRadius) {
            erase.push_back(contours[i]);
        }
    }
    
    // 
    // dest = Mat::zeros(bw.size(), bw.type());
    dest = src.clone();
    cv::drawContours(dest, erase, -1, cv::Scalar::all(0), CV_FILLED);    
}

// Get binary thresholded image
// low_HSV, hi_HSV - low, high range values for threshold as a list [H,S,V]
// debug= True to display the binary image generated
void getBinary(const cv::Mat& src, cv::Scalar& low_HSV, cv::Scalar& hi_HSV, cv::Mat& dest) {
    cv::Mat frame = src.clone();
    cv::cvtColor(frame, frame, CV_BGR2HSV);
    cv::Mat bw;
    cv::inRange(frame, low_HSV, hi_HSV, bw);
    
    removeSmall(bw, dest, 10);
}

template<template <typename> class P = std::less >
struct compare_pair_second {
    template<class T1, class T2> bool operator()(const std::pair<T1, T2>& left, const std::pair<T1, T2>& right) {
        return P<T2>()(left.second, right.second);
    }
};

std::vector<std::pair<Point, int> > LaneExtractCv::findLanes(const Mat& img, int boxHeight, int boxWidth) {
    int boxWidthHalf = boxWidth/2;
    cv::Size size = img.size();
    printf("Size: %d, %di\n", size.height, size.width);
    int rTop = size.height * 19 / 20; // - boxHeight - 20;
    int startX = -1;
    int midLine = size.width / 2;
    std::vector<std::pair<Point, int> > pointsWithScore;

    int laneEdgeThreshold = 5000;
    //slide a window across the bottom of the image to try and find the area with the biggest split between dense
    // and not dense...this would be a good place to look for edges
    //TODO: catch and handle exceptions!@
    for (int c = 0; c < size.width - boxWidth; c++) {
        Rect rectL = Rect(c,                rTop, boxWidthHalf, boxHeight);
        Rect rectR = Rect(c + boxWidthHalf, rTop, boxWidthHalf, boxHeight);
        Mat tileL = Mat(img, rectL);
        Mat tileR = Mat(img, rectR);
        cv::Scalar diff = sum(tileL - tileR);
        //c is the start column...we want the midpoint of the box
        int potential = c + boxWidthHalf - 1;
        int score = abs(diff.val[0]);
        if (score > laneEdgeThreshold) { 
            printf("%f\n", abs(diff.val[0]));
            pointsWithScore.push_back(std::pair<Point, int>(Point(potential, rTop + boxHeight/2), score));
        }
    }

    return pointsWithScore;
}

Lane LaneExtractCv::extractLane(const cv::Mat& img, cv::Point initialPoint, int boxHeight, int boxWidth) {
    CurveDetect cd = CurveDetect(boxHeight, boxWidth);
    cd.fitCurve(img, initialPoint, 20);
    return Lane(cd.getPointsOnCurve());
}

void LaneExtractCv::annotateImage(cv::Mat& img, const Lane& lane) {
    std::vector<Point>::const_iterator it = lane.getPoints().begin();
    Point pt = *it;
    while (++it != lane.getPoints().end()) {
        Point nextPt = *it;
        printf ("Line from (%d, %d) to (%d, %d)\n", pt.x, pt.y, it->x, it->y);
        line(img, pt, nextPt, Scalar(255, 0, 0), 2);
        pt = nextPt;
    }
}
void LaneExtractCv::doLaneExtraction(const cv::Mat& src, const GroundTransform& _gtrans, int maxLanes) {
    
    cv::Mat srcGray, mask, thold, edge, edgeMask;
    cv::cvtColor(src, srcGray, CV_BGR2GRAY);
    //yellow threshold
    getBinary(src, LOW_HSV, HIGH_HSV, mask);
    // edge detection
    cannyThreshold(src, srcGray, edge, edgeMask);
    // removeSmall(edgeMask, edgeMask, 50);
    //binary threshold for the really bright stuff
    threshold( srcGray, thold, 220, 255, 0);
    // printf("%d, %d, %d", thold.size().width, thold.size().height, thold.channels());
    // printf("%d, %d, %d", maskA.size().width, maskA.size().height, maskA.channels());
    mask = mask + thold + edgeMask;
    //remove small features (probably not lanes)
    removeSmall(mask, mask, 100);

	GroundTransformProjective gtrans;

    Mat proj;
    gtrans.transform(mask, proj, 0.0);

	Mat isolated;
	const double angle_tweak = 0.275 * laneIsolate(isolated, proj);

	cout << angle_tweak << endl;

	gtrans.transform(mask, proj, angle_tweak);
	laneIsolate(isolated, proj);

	this->processed = proj;
	return;
   
    //create a color image, to be annotated with information about the lane
    // extraction 
    cvtColor(proj, this->processed, CV_GRAY2BGR);

    //find lanes in resulting transform and fill in this->lanes 
    int boxHeight = 20;
    int boxWidth = 10;
    std::vector<std::pair<Point, int> > pointsWithScore = findLanes(proj, boxHeight, boxWidth);

    std::sort(pointsWithScore.begin(), pointsWithScore.end(), compare_pair_second<std::less>());
    std::vector<cv::Point> lanePoints;
    int ii = 0;
    //filter the lanes down, and annotate the image
    for (std::vector<std::pair<cv::Point, int> >::iterator it = pointsWithScore.begin();
         it != pointsWithScore.end();
         it++, ii++) {
        if (ii < maxLanes) {
            lanePoints.push_back(it->first);
        }
        circle(this->processed, it->first, 2, Scalar(0, 0, 255), 2);
    }
    
    for (std::vector<Point>::iterator it = lanePoints.begin();
         it != lanePoints.end();
         it++) {
        Point start = *it;
        printf("Potential lane at (%d, %d)\n", start.x, start.y);
        circle(this->processed, start, 2, Scalar(0, 255, 0), 2);
        double t = (double)getTickCount();
        try {
            //TODO: this is currently pretty slow
            Lane lane = extractLane(proj, start, boxHeight, boxWidth);
            lanes.push_back(lane);
            annotateImage(this->processed, lane);
        } catch (cv::Exception& ex) {
            printf("Could not detect lane at (%d, %d)!\n", start.x, start.y);
        }
        t = ((double)getTickCount() - t)/getTickFrequency();
        cout << "Times passed in seconds: " << t << endl;
    }
}
