
#include "ground_transform_projective.h"

using namespace ld;
using namespace cv;

void GroundTransformProjective::transform(const Mat& input, Mat& output) const {
    CvSize input_size = input.size;

    // resolution of the input image in the camera frame
    const int input_x_res = input_size.width;
    const int input_y_res = input_size.height;

    // resolution of the output image in the ground frame
    const int output_x_res = 800;
    const int output_y_res = 800;

    // dimensions of the ground frame we are sampling from
    const double ground_output_y_dim = 30.0; // ft
    const double ground_output_x_dim = 30.0; // ft

    // general scale factor for the camera frame the input is in
    const double camera_scale = 1.0; // pixels/focal-length -- ADJUST ME
    const double camera_pitch = M_PI_2; // radians from vertical -- ADJUST ME

    ground = Mat::zeros(cvSize(x_res, y_res), src.type());
    
    for (int output_x = 0; output_x < output_x_res; output_x++) {
        for (int output_y = 0; y < output_y_res; output_y++) {

            // ground frame coordinates of sample point (in ft, it's all relative)
            const double x_ground = (output_x / output_x_res) * ground_output_x_dim - (ground_output_x_dim / 2);
            const double y_ground = (output_y / output_y_res) * ground_output_y_dim;
            const double z_ground = - 21.0 / 12.0;

            // camera frame (post-transform) coordinates of ground sample point
            const double x_camera = camera_scale * x_ground;
            const double y_camera = camera_scale * (y_ground * cos(camera_pitch) - z_ground * sin(camera_pitch));
            const double z_camera = y_ground * sin(camera_pitch) + z_ground * cos(camera_pitch);

            // input image coordinates
            int input_x = (x_camera / z_camera) + (input_x_res / 2);
            int input_y = (y_camera / z_camera) + (input_y_res / 2);
            
            if (input_y < 0 || input_y >= input_y_res || input_x < 0 || input_x >= input_x_res) {
                continue;
            }

            output.at<unsigned char>(output_y_res - output_y, output_x_res - output_x) =
                input.at<unsigned char>(input_y, input_x);
        }
    }
}
