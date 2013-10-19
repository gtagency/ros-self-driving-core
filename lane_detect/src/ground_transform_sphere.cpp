
#include "ground_transform_sphere.h"

using namespace ld;
using namespace cv;

void GroundTransformSphere::transform(const Mat& src, Mat& ground) const {
	CvSize srcSize = src.size();
    int width = srcSize.width;
    int height = srcSize.height;
    
    int x_res = 800;
    int y_res = 800;

    ground = Mat::zeros(cvSize(x_res, y_res), src.type());

    double groundWidth = 30.0; //ft
    double groundHeight = 30.0; //ft
    double h = 21.0/12.0; //ft
    double deg_fov = (120.0 / 180.0) * M_PI;
    double phi_scale = width / deg_fov;
    double theta_scale = height / deg_fov * 1.0;
    double x_frame_off = x_res/2;
    double y_frame_off = y_res/10;
    double phi_off = -1.15;
    double theta_off = -1.55;
    
    for (int x = 0; x < x_res; x++) {
        for (int y = 0; y < y_res; y++) {
            double x_ground = x * groundWidth / x_res - (groundWidth / 2);
            double y_ground = y * groundHeight / y_res;
            
			// angle from the camera down to the point on the ground: acos (-h / r), r = sqrt(x^2 + y^2 + h^)
            double theta = acos(-h/ sqrt(pow(x_ground, 2) + pow(y_ground, 2) + pow(h, 2)));
			// angle on the ground relative to the origin (under the camera)
            double phi = atan2(y_ground, x_ground);
			//convert phi and theta to image frame coordinates...this applies
			// camera calibration adjustments
            int x_frame = (phi + phi_off) * phi_scale + x_frame_off;
            int y_frame = (theta + theta_off) * theta_scale + y_frame_off;
            // if (x == 0 && y == 0) {
            //     printf("%d, %d, %f, %f, %f, %f\n", x_frame, y_frame, theta, phi, theta_scale, phi_scale);
            // }
            if (y_frame >=  0 && y_frame < height && x_frame >= 0 && x_frame < width) {
                // printf("%d, %d, %f\n", x, y,  src.at<Vec3b>(y_frame, x_frame));
                
                // ground.at<Vec3b>(y_res-y, x_res-x) = src.at<Vec3b>(y_frame, x_frame);
                ground.at<unsigned char>(y_res-y, x_res-x) = src.at<unsigned char>(y_frame, x_frame);
                
            }
        }
    }
    
}
