
#include "ground_transform_projective.h"

using namespace ld;
using namespace cv;

void GroundTransformProjective::transform(const Mat& input, Mat& output) const {
	this->transform(input, output);
}

void GroundTransformProjective::transform(const Mat& input, Mat& output, double angle_tweak) const {

    // resolution of the input image in the camera frame
    const Size input_size = input.size();
    const int input_x_res = input_size.width;
    const int input_y_res = input_size.height;

    // resolution of the output image in the ground frame
    const int output_x_res = 800;
    const int output_y_res = 400;

    // dimensions of the ground frame we are sampling from
    // ground frame is the rectangle from
    //   (-ground_output_x_dim / 2, 0)
    //   to (ground_output_x_dim, ground_output_y_dim)
    //   i.e. (-15 ft, 0 ft) to (15 ft, 30 ft)
    const double ground_output_y_dim = 30.0; // ft
    const double ground_output_x_dim = 60.0; // ft

    // general scale factor for the camera frame the input is in
    const double camera_scale = 1000.0; // pixels/focal-length -- ADJUST ME
    const double camera_pitch = M_PI_2 + .26 + angle_tweak; // radians from vertical -- ADJUST ME

    output = Mat::zeros(Size(output_x_res, output_y_res), input.type());
printf("wee\n");    
    // compute trig outside of the loop for such fastness
    const double cos_camera_pitch = cos(camera_pitch);
    const double sin_camera_pitch = sin(camera_pitch);
    
    for (int output_x = 0; output_x < output_x_res; output_x++) {
        for (int output_y = 0; output_y < output_y_res; output_y++) {

            // ground frame coordinates of sample point (in ft, it's all relative)
            const double x_ground = ((double) output_x / output_x_res) * ground_output_x_dim - (ground_output_x_dim / 2);
            const double y_ground = ((double) output_y / output_y_res) * ground_output_y_dim;
            const double z_ground = - 21.0 / 12.0;

            // camera frame (post-transform) coordinates of ground sample point
            const double x_camera = x_ground;
            const double y_camera = y_ground * cos_camera_pitch - z_ground * sin_camera_pitch;
            const double z_camera = y_ground * sin_camera_pitch + z_ground * cos_camera_pitch;

            if (z_camera <= 0.0) {
                // don't divide by zero or look behind the camera
                continue;
            }

            // input image coordinates
            const int input_x = camera_scale * (x_camera / z_camera) + (input_x_res / 2);
            const int input_y = camera_scale * (y_camera / z_camera) + (input_y_res / 2);
            
            if (input_y < 0 || input_y >= input_y_res || input_x < 0 || input_x >= input_x_res) {
                // don't sample outside the camera frame
				output.at<unsigned char>(output_y, output_x_res - output_x - 1) = 0x80;
				continue;
            }
            
            output.at<unsigned char>(output_y, output_x_res - output_x - 1) =
                input.at<unsigned char>(input_y, input_x) ? 0xFF : 0;
        }
    }
printf("weeee\n");    
/*
    // resolution of the input image in the camera frame
    const Size input_size = input.size();
    const int input_x_res = input_size.width;
    const int input_y_res = input_size.height;

    // resolution of the output image in the ground frame
    const int output_x_res = 800;
    const int output_y_res = 800;

    // dimensions of the ground frame we are sampling from
    // ground frame is the rectangle from
    //   (-ground_output_x_dim / 2, 0)
    //   to (ground_output_x_dim, ground_output_y_dim)
    //   i.e. (-15 ft, 0 ft) to (15 ft, 30 ft)
    const double ground_output_y_dim = 30.0; // ft
    const double ground_output_x_dim = 30.0; // ft

    // general scale factor for the camera frame the input is in
    const double camera_scale = 500.0; // pixels/focal-length -- ADJUST ME
    const double camera_pitch = M_PI_2 + .59; // radians from vertical -- ADJUST ME

    output = Mat::zeros(Size(output_x_res, output_y_res), input.type());
    
    // compute trig outside of the loop for such fastness
    const double cos_camera_pitch = cos(camera_pitch);
    const double sin_camera_pitch = sin(camera_pitch);
    
    for (int output_x = 0; output_x < output_x_res; output_x++) {
        for (int output_y = 0; output_y < output_y_res; output_y++) {

            // ground frame coordinates of sample point (in ft, it's all relative)
            const double x_ground = ((double) output_x / output_x_res) * ground_output_x_dim - (ground_output_x_dim / 2);
            const double y_ground = ((double) output_y / output_y_res) * ground_output_y_dim;
            const double z_ground = - 21.0 / 12.0;

            // camera frame (post-transform) coordinates of ground sample point
            const double x_camera = x_ground;
            const double y_camera = y_ground * cos_camera_pitch - z_ground * sin_camera_pitch;
            const double z_camera = y_ground * sin_camera_pitch + z_ground * cos_camera_pitch;

            if (z_camera <= 0.0) {
                // don't divide by zero or look behind the camera
                continue;
            }

            // input image coordinates
            const int input_x = camera_scale * (x_camera / z_camera) + (input_x_res / 2);
            const int input_y = camera_scale * (y_camera / z_camera) + (input_y_res / 2);
            
            if (input_y < 0 || input_y >= input_y_res || input_x < 0 || input_x >= input_x_res) {
                // don't sample outside the camera frame
                continue;
            }
            
//            output.at<Vec3b>(output_y_res - output_y, output_x) =
  //              input.at<Vec3b>(input_y, input_x);

             output.at<unsigned char>(output_y_res - output_y, output_x) = input.at<unsigned char>(input_y, input_x);
        }
    } */
}
