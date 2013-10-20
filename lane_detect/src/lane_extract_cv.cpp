
#include <vector>
#include "lane_extract_cv.h"

using namespace ld;
using namespace std;

LaneExtractCv::LaneExtractCv(const cv::Mat& src, const GroundTransform& gtrans) {
    doLaneExtraction(src, gtrans);
}

LaneExtractCv::~LaneExtractCv() {
}

int LaneExtractCv::numLanes() {
   return lanes.size();
}

void LaneExtractCv::describeLane(int num, Lane& lane) {
}

const cv::Mat& LaneExtractCv::getProcessedImage() {
   return this->processed; 
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
    vector<vector<cv::Point>> contours;
    cv::findContours(src.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    vector<vector<cv::Point>> erase;

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<cv::Point>> contours_poly( contours.size() );
    // vector<Rect> boundRect( contours.size() );
    // vector<Point2f>center( contours.size() );
    // vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ) {
        cv::Point2f center;
        float radius;
        cv::approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
        // boundRect[i] = boundingRect( Mat(contours_poly[i]) );
        cv::minEnclosingCircle( (Mat)contours_poly[i], center, radius );
        if (radius < minRadius) {
            erase.push_back(contours[i]);
        }
    }
    
    // 
    // dest = Mat::zeros(bw.size(), bw.type());
    dest = src.clone();
    cv::drawContours(dest, erase, -1, Scalar::all(0), CV_FILLED);    
}

// Get binary thresholded image
// low_HSV, hi_HSV - low, high range values for threshold as a list [H,S,V]
// debug= True to display the binary image generated
void getBinary(const cv::Mat& src, cv::Scalar& low_HSV, cv::Scalar& hi_HSV, cv::Mat& dest) {
    cv::Mat frame = src.clone();
    cv::cvtColor(frame, frame, CV_BGR2HSV);
    cv::Mat bw;
    cv::inRange(frame, low_HSV, hi_HSV, bw);
    
    cv::removeSmall(bw, dest, 10);
}
            


// void lensTransform(Point& pt, double h, double cameraAngle, double& vec[2]) {
// }
// void groundTransform(Point& framePt, Point& groundPt, double h, double cameraAngle) {
//     double vec[2];
//     lensTransform(framePt, h, cameraAngle, vec);
//     double x = sqrt( (pow(-h / cos(vec[0]), 2) - pow(h, 2))
//                       / (1 + pow(tan(vec[1]), 2)) );
//     double y = x * tan(phi);
//     
// }

// void groundTransform(Mat& src, Mat& ground) {
//     int width = 1280;
//     int height = 720;
//     
//     int x_res = 400;
//     int y_res = 400;
// 
//     ground = Mat::zeros(cvSize(x_res, y_res), src.type());
// 
//     double groundWidth = 20.0; //ft
//     double groundHeight = 20.0; //ft
//     double h = 21.0/12.0; //ft
//     double deg_fov = 120.0;
//     double phi_scale = 100000 * (M_PI / 180) * (deg_fov / width);
//     double theta_scale = 100000 * (M_PI / 180) * (deg_fov / width);
//     int x_frame_off = width/2;
//     int y_frame_off = 0;
//     
//     for (int x = 0; x < x_res; x++) {
//         for (int y = 0; y < y_res; y++) {
//             double x_ground = x / groundWidth - (groundWidth/2);
//             double y_ground = y / groundHeight;
//             
//             double theta = acos(-h/ sqrt(pow(x_ground, 2) + pow(y_ground, 2) + pow(h, 2)));
//             double phi = atan2(y_ground, x_ground);
//             int x_frame = phi * phi_scale + x_frame_off;
//             int y_frame = theta * theta_scale + y_frame_off;
//             ground.at<double>(y, x) = src.at<double>(y_frame, x_frame);
//             if (x == 0 && y == 0) {
//                 printf("%d, %d, %f, %f, %f, %f, %f\n", x_frame, y_frame, src.at<double>(y_frame, x_frame), theta, phi, theta_scale, phi_scale);
//             }
//         }
//     }   
// }

void LaneExtractCv::doLaneExtraction(const cv::Mat& src, const GroundTransform& gtrans) {
    
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

    gtrans.transform(src, this->processed);
    
    //find lanes in resulting transform and fill in this->lanes 
}
