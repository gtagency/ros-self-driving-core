
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <core_msgs/Obstacle.h>
#include <core_msgs/ObstacleArrayStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "lane_extract_cv.h"
#include "ground_transform_sphere.h"

namespace enc = sensor_msgs::image_encodings;

using namespace core_msgs;
using namespace ld;

ros::Publisher lanePoly_pub;
image_transport::Publisher projImage_pub;
/*
void projConnectCallback(const ros::SingleSubscriberPublisher&) {
  ROS_INFO("connectCallback");
}

void projDisconnectCallback(const ros::SingleSubscriberPublisher&) {
  ROS_INFO("disconnectCallback");
}
*/
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    GroundTransformSphere gtrans;
    LaneExtractCv le(cv_ptr->image, gtrans);

    //publish the processed/projected image if there are
    // any listeners
    if (projImage_pub.getNumSubscribers() > 0) {
        ROS_INFO("Publishing processed image");
        cv_bridge::CvImage projImage;
//        projImage.header 
        projImage.image = le.getProcessedImage();
        projImage.encoding = enc::BGR8; //"passthrough"; //le.getProcessedImageEnc();
        projImage_pub.publish(projImage.toImageMsg());
    }
    
    ObstacleArrayStamped msg;
    //fill in timestamp info TODO
    for (int ii = 0; ii < le.numLanes(); ii++) {
        Lane lane;
        le.describeLane(ii, lane);
        
        Obstacle obstacle;
        geometry_msgs::Polygon poly;
        for (std::vector<Point>::const_iterator it = lane.getPoints().begin();
             it != lane.getPoints().end();
             ++it) {
            geometry_msgs::Point32 p32;
            p32.x = it->getX();
            p32.y = it->getY();
            p32.z = 0;
            poly.points.push_back(p32);
        }
        obstacle.type = Obstacle::TYPE_LANE;
        obstacle.polygons.push_back(poly);
        msg.obstacles.push_back(obstacle);
    }
    lanePoly_pub.publish(msg);
}

int main(int argc, char** argv) {
  
    ros::init(argc, argv, "lane_detect");
  
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    //ros::NodeHandle n_params("~");
    //n_params.getParam("
   
    projImage_pub = it.advertise("image_projection", 1);
    image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
    
    lanePoly_pub  = nh.advertise<ObstacleArrayStamped>("obstacles", 100);
    
    ros::spin();
}
