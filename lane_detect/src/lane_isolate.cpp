
#include <iostream>
#include <vector>
#include <opencv2/contrib/contrib.hpp>

#include "lane_isolate.h"

using namespace std;
using namespace cv;

namespace lane_isolate {


static void addEndpoints(const Vec4f& line, std::vector<Point>& points) {
	const double yint1 = 0;
	const double xint1 = line[2] + (yint1 - line[3]) / line[1] * line[0];
	
	const double yint2 = line[3];
	const double xint2 = line[2];

    points.push_back(Point(xint1, yint1));
    points.push_back(Point(xint2, yint2));
	//cv::line(img, Point(xint1, yint1), Point(xint2, yint2), color, 1, 8);
}

void maskImageByDensity(Mat& img) {
    int boxHeight = 20;
    int boxWidth = 40;
    int boxHeightHalf = boxHeight >> 2;
    int boxWidthHalf = boxWidth >> 2;
//    double t = (double)getTickCount();
    cv::Size size = img.size();
    Mat img2 = img.clone();
//    cvtColor(img2, img2, CV_GRAY2BGR);

    int step = 3;
    int maxDensity = boxHeight * boxWidth;
    unsigned char *ptr = img2.ptr<unsigned char>(0); //assumed continuous
    for (int r = 0; r < size.height - boxHeight; r += step) {
        for (int c = 0; c < size.width - boxWidth; c += step) {
            Rect rect = Rect(c, r, boxWidth, boxHeight);
            Mat tile = Mat(img, rect);
            int  density = sum(tile).val[0];
            //place this density in the center of the window
            int index = (r + boxHeightHalf) * size.width + boxWidthHalf + c; //(r + bh) * size.width + c + bw;
            int pxVal = (int)density / maxDensity; //densities.push_back(std::pair<Point, int>(pt, density));
            if (pxVal < 50) {
                for (int ii = 0; ii < step; ii++) {
                    for (int jj = 0; jj < step; jj++) {
                        ptr[index + ii * size.width + jj] = 0;
  //                      img2.at<Vec3b>(r+boxHeightHalf+ii, c+boxWidthHalf+jj)[0] = 0;
    //                    img2.at<Vec3b>(r+boxHeightHalf+ii, c+boxWidthHalf+jj)[1] = 0;
                    }
                }
            }
        }
    }
        
  //      t = ((double)getTickCount() - t)/getTickFrequency();
    //    cout << "Times passed in seconds: " << t << endl;
    img = img2; // * maxDensity;
}

static void drawLine(Mat& img, const Vec4f& line, Scalar color) {
	const double yint1 = 0;
	const double xint1 = line[2] + (yint1 - line[3]) / line[1] * line[0];
	
	const double yint2 = line[3];
	const double xint2 = line[2];

	cv::line(img, Point(xint1, yint1), Point(xint2, yint2), color, 1, 8);
}

double laneIsolate(Mat& output, const Mat& input) {
	const Size input_size = input.size();
	Mat filled = input.clone();

    //cvtColor(input, output, CV_GRAY2BGR);
    maskImageByDensity(filled);
//    return 0;
	// flood fill potential lane
	const Point seedPoint(input_size.width / 2, input_size.height / 8);
	floodFill(filled, seedPoint, 0xC0);

	// separate filled into unknown (0x80), lane (0xFF), and not lane (0x00)
	for (int y = 0; y < filled.size().height; y++) {
		for (int x = 0; x < filled.size().width; x++) {
			if (filled.at<unsigned char>(y, x) == 0xC0) {
				filled.at<unsigned char>(y, x) = 0xFF;
			}
			else if (filled.at<unsigned char>(y, x) == 0x80) {
				filled.at<unsigned char>(y, x) = 0x80;
			}
			else {
				filled.at<unsigned char>(y, x) = 0x00;
			}
		}
	}

	vector<Point> right_edge_points;
	vector<Point> left_edge_points;
    vector<Point> all_points;

	// find edges of lane
	output = Mat::zeros(filled.size(), filled.type());	
	for (int y = filled.size().height / 8; y < filled.size().height; y++) {
		int smooth = 0;
		for (int x = 0; x < filled.size().width; x++) {
			if (smooth) {
				smooth--;
				continue;
			}
			if (filled.at<unsigned char>(y, x+0) == 0x00 &&
				filled.at<unsigned char>(y, x+1) == 0xFF) {

				output.at<unsigned char>(y, x) = 0xFF;
                Point pt = Point(x, y);
                all_points.push_back(pt);
				if (right_edge_points.size() < 350) {
					right_edge_points.push_back(pt);
				}
				smooth = 5;
			}
			else if (filled.at<unsigned char>(y, x+0) == 0xFF &&
					 filled.at<unsigned char>(y, x+1) == 0x00) {

				output.at<unsigned char>(y, x) = 0x80;
                Point pt = Point(x, y);
                all_points.push_back(pt);
				if (left_edge_points.size() < 350) {
					left_edge_points.push_back(pt);
				}
				break;
			}
		}
	}


	output.at<unsigned char>(0, output.size().width / 2) = 0xC0;

	Vec4f right_edge;
	if (right_edge_points.size() > 10) {
		fitLine(right_edge_points, right_edge, CV_DIST_L2, 0, 0.01, 0.01);
        addEndpoints(right_edge, all_points);	
		drawLine(output, right_edge, 0x80);
    }
	Vec4f left_edge;
	if (left_edge_points.size() > 10) {
		fitLine(left_edge_points, left_edge, CV_DIST_L2, 0, 0.01, 0.01);
        addEndpoints(left_edge, all_points);	
		drawLine(output, left_edge, 0x80);
	}
   /* 
    vector<int> hull;
    convexHull(Mat(all_points), hull, CV_CLOCKWISE);
    int i;
    //for( i = 0; i < points.size(); i++ )
      //  circle(output, points[i], 3, Scalar(0xC0), CV_FILLED, CV_AA);
        
    int hullcount = (int)hull.size();
    Point pt0 = all_points[hull[hullcount-1]];
       
    for( i = 0; i < hullcount; i++ )
    {
        Point pt = all_points[hull[i]];
        circle(output, pt, 2, Scalar(0xC0), CV_FILLED, CV_AA);
        line(output, pt0, pt, Scalar(0xC0), 1, CV_AA);
        pt0 = pt;
    }


    */

    vector<vector<cv::Point> > contours;
    cv::findContours(output.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    vector<vector<cv::Point> > erase;

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<cv::Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    // vector<Point2f>center( contours.size() );
    // vector<float>radius( contours.size() );

    cvtColor(output, output, CV_GRAY2BGR);
    int boxWidth = 20;
    int boxHeight = 40;
    for( int i = 0; i < contours.size(); i++ ) {
        cv::Point2f center;
        float radius;
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );

        vector<Point> tmp = contours_poly.at(i);
        const Point* elementPoints[1] = { &tmp[0] };
        int numberOfPoints = (int)tmp.size();

        fillPoly(output, elementPoints, &numberOfPoints, 1, Scalar(0, 255, 0)); 
        int j = 0;
        for (std::vector<Point>::iterator it = contours_poly[i].begin();
            it != contours_poly[i].end();
            it++, j++) {
            
            circle(output, *it, 2, Scalar(255 - j * 2, 0, 0), CV_FILLED, CV_AA);
        }
break;
//        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
  //      rectangle( output, boundingRect[i].tl(), boundingRect[i].br(), color, 2, 8, 0 );
//        cv::minEnclosingCircle( (cv::Mat)contours_poly[i], center, radius );
  //      if (radius < minRadius) {
            erase.push_back(contours[i]);
      //  }
    }
    cout << "Contours: " << contours.size() << endl; 
    // 
    // dest = Mat::zeros(bw.size(), bw.type());
   //cv::drawContours(output, erase, -1, Scalar(0, 0, 255), CV_FILLED); 

	if (right_edge_points.size() > 10 && left_edge_points.size() > 10) {
		return right_edge[0] - left_edge[0];
	}

	return 0.0;
}

}
