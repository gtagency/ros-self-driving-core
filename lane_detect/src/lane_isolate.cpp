
#include <iostream>
#include <vector>

#include "lane_isolate.h"

using namespace std;
using namespace cv;

namespace lane_isolate {

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
				if (right_edge_points.size() < 50) {
					right_edge_points.push_back(Point(x, y));
				}
				smooth = 5;
			}
			else if (filled.at<unsigned char>(y, x+0) == 0xFF &&
					 filled.at<unsigned char>(y, x+1) == 0x00) {

				output.at<unsigned char>(y, x) = 0x80;
				if (left_edge_points.size() < 50) {
					left_edge_points.push_back(Point(x, y));
				}
				break;
			}
		}
	}

	output.at<unsigned char>(0, output.size().width / 2) = 0xC0;

	Vec4f right_edge;
	if (right_edge_points.size() > 10) {
		fitLine(right_edge_points, right_edge, CV_DIST_L2, 0, 0.01, 0.01);
		drawLine(output, right_edge, 0xFF);
	}

	Vec4f left_edge;
	if (left_edge_points.size() > 10) {
		fitLine(left_edge_points, left_edge, CV_DIST_L2, 0, 0.01, 0.01);
		drawLine(output, left_edge, 0x80);
	}

	if (right_edge_points.size() > 10 && left_edge_points.size() > 10) {
		return right_edge[0] - left_edge[0];
	}

	return 0.0;
}

}
