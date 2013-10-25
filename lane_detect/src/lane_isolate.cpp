
#include <iostream>
#include <vector>
#include <opencv2/contrib/contrib.hpp>

#include "lane_isolate.h"

using namespace std;
using namespace cv;

namespace lane_isolate {

//TODO: REFACTOR THE HELL OUT OF THIS

static void addEndpoints(const Vec4f& line, std::vector<Point>& points, bool front) {
	const double yint1 = 0;
	const double xint1 = line[2] + (yint1 - line[3]) / line[1] * line[0];
	
	const double yint2 = line[3];
	const double xint2 = line[2];

    if (front) {
        std::vector<Point> new_points;
        new_points.push_back(Point(xint1, yint1));
        new_points.insert(new_points.end(), points.begin(), points.end());
        points = new_points;
    } else {
        points.push_back(Point(xint1, yint1));
    }
//    points.push_back(Point(xint2, yint2));
	//cv::line(img, Point(xint1, yint1), Point(xint2, yint2), color, 1, 8);
}

void maskImageByDensity(Mat& img) {
    int boxHeight = 20;
    int boxWidth = 20;
    int boxHeightHalf = boxHeight >> 2;
    int boxWidthHalf = boxWidth >> 2;
    cv::Size size = img.size();
    Mat img2 = img.clone();

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
        
    img = img2; // * maxDensity;
}

static void drawLine(Mat& img, const Vec4f& line, Scalar color) {
	const double yint1 = 0;
	const double xint1 = line[2] + (yint1 - line[3]) / line[1] * line[0];
	
	const double yint2 = line[3];
	const double xint2 = line[2];

	cv::line(img, Point(xint1, yint1), Point(xint2, yint2), color, 1, 8);
}

double euclidean(Point pt1, Point pt2) {
    return sqrt(pow(pt2.x - pt1.x, 2) + pow(pt2.y - pt1.y, 2));
}
double laneIsolate(const Mat& input, Mat& output, std::vector<std::vector<Point> >& polygons) {
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
//				if (right_edge_points.size() < 150) {
					right_edge_points.push_back(pt);
//				}
				smooth = 5;
			}
			else if (filled.at<unsigned char>(y, x+0) == 0xFF &&
					 filled.at<unsigned char>(y, x+1) == 0x00) {

				output.at<unsigned char>(y, x) = 0x80;
                Point pt = Point(x, y);
                all_points.push_back(pt);
//				if (left_edge_points.size() < 150) {
					left_edge_points.push_back(pt);
//				}
				break;
			}
		}
	}


	output.at<unsigned char>(0, output.size().width / 2) = 0xC0;
    std::vector<Point> some_points = right_edge_points;
//    some_points.insert(some_points.end(), left_edge_points.begin(), left_edge_points.end());
	Vec4f right_edge;
	if (right_edge_points.size() > 10) {
		fitLine(right_edge_points, right_edge, CV_DIST_L2, 0, 0.01, 0.01);
        addEndpoints(right_edge, right_edge_points, true);	
		drawLine(output, right_edge, 0x80);
    }
	Vec4f left_edge;
	if (left_edge_points.size() > 10) {
		fitLine(left_edge_points, left_edge, CV_DIST_L2, 0, 0.01, 0.01);
        addEndpoints(left_edge, left_edge_points, true);	
		drawLine(output, left_edge, 0x80);
	}

    vector<vector<cv::Point> > contours;
    cv::findContours(output.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    /// Approximate contours to polygons + get bounding rects and circles
    vector<vector<cv::Point> > contours_poly( contours.size() );
    std::vector<Point> right_poly, left_poly;
    cvtColor(filled, output, CV_GRAY2BGR);
    //output = Mat::zeros(output.size(), output.type());
    cv::approxPolyDP( cv::Mat(left_edge_points),  left_poly,  2, true );
    cv::approxPolyDP( cv::Mat(right_edge_points), right_poly, 2, true );

    std::vector<Point> full_poly = right_poly;
    Point firstPt = full_poly[0];
    Point testPt1 = left_poly[0];
    Point testPt2 = left_poly[1];
   
    //TODO: may need to be more sophisticated...section parts of the image, or something
    std::reverse(left_poly.begin(), left_poly.end());
    full_poly.insert(full_poly.end(), left_poly.begin(), left_poly.end());
    
    int j = 0;
    //annotate the output image
    Point lastPt;
    for (std::vector<Point>::iterator it = full_poly.begin();
        it != full_poly.end();
        it++, j++) {
        
        if (j > 0) {
	        cv::line(output, lastPt, *it, Scalar(0, 255, 255 - j * 2), 1, 8);
        }
        circle(output, *it, 2, Scalar(0, 255, 255 - j * 2), CV_FILLED, CV_AA);
        lastPt = *it;
    }
    
	cv::line(output, lastPt, full_poly[0], Scalar(0, 255, 255 - j * 2), 1, 8);
    polygons.clear();
    polygons.push_back(full_poly);
    return 0;

//this code is more sophisticated...and may need to be revisted if the above doesnt work
// but it works well for now
#if 0
    for( int i = 0; i < contours.size(); i++ ) {
        cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );

        //vector<Point> tmp = contours_poly.at(i);
        //const Point* elementPoints[1] = { &tmp[0] };
        //int numberOfPoints = (int)tmp.size();

//        fillPoly(output, elementPoints, &numberOfPoints, 1, Scalar(0, 255, 0)); 
        int j = 0;
        //annotate the output image
        Point lastPt;
        for (std::vector<Point>::iterator it = contours[i].begin();
            it != contours[i].end();
            it++, j++) {
            
            if (j > 0) {
	            cv::line(output, lastPt, *it, Scalar(0, 0, 255 - j * 2), 1, 8);
            } else {
                circle(output, *it, 2, Scalar(0, 0, 255 - j * 2), CV_FILLED, CV_AA);
            }
            lastPt = *it;
        }
        j = 0;
        for (std::vector<Point>::iterator it = contours_poly[i].begin();
            it != contours_poly[i].end();
            it++, j++) {
            
            if (j > 0) {
	            cv::line(output, lastPt, *it, Scalar(255 - j * 2, 0, 0), 1, 8);
            } else {
                circle(output, *it, 2, Scalar(255 - j * 2, 0, 0), CV_FILLED, CV_AA);
            }
            lastPt = *it;
        }
    }
    cout << "Contour polygons: " << contours_poly.size() << endl; 

    polygons = contours_poly;
	if (right_edge_points.size() > 10 && left_edge_points.size() > 10) {
		return right_edge[0] - left_edge[0];
	}
#endif
	return 0.0;
}

}
