
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
//#include "ground_transform_sphere.h"
#include "ground_transform_projective.h"

#include "cv_bridge_ex.h"

namespace enc = sensor_msgs::image_encodings;

using namespace core_msgs;
using namespace ld;

ros::Publisher lanePoly_pub;
image_transport::Publisher procImage_pub;
/*
void projConnectCallback(const ros::SingleSubscriberPublisher&) {
  ROS_INFO("connectCallback");
}

void projDisconnectCallback(const ros::SingleSubscriberPublisher&) {
  ROS_INFO("disconnectCallback");
}
*/

geometry_msgs::Point32 cvtCvPointToROSPoint(const cv::Point& point) {
    geometry_msgs::Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
    point32.z = 0;
    return point32;
}

std::vector<geometry_msgs::Polygon> buildPolygonsFromLane(const Lane& lane) {
    std::vector<geometry_msgs::Polygon> polys;
    std::vector<cv::Point>::const_iterator it = lane.getPoints().begin();
    cv::Point pt = *it;
    while (++it != lane.getPoints().end()) {
        geometry_msgs::Polygon poly;
        cv::Point nextPt = *it;
        poly.points.push_back(cvtCvPointToROSPoint(pt));
        poly.points.push_back(cvtCvPointToROSPoint(nextPt));
        polys.push_back(poly);
        pt = nextPt;
    }
    return polys;
}
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
  
    int maxLanes = 3;  
    //GroundTransformSphere gtrans;
    GroundTransformProjective gtrans;
    LaneExtractCv le(cv_ptr->image, gtrans, maxLanes);

    //publish the processed/projected image if there are
    // any listeners
    if (procImage_pub.getNumSubscribers() > 0) {
        ROS_INFO("Publishing processed image");
        cv_bridge::CvImage procImage;
//        projImage.header 
        procImage.image = le.getProcessedImage();
        procImage.encoding = getRosType(le.getProcessedImageEnc());
        procImage_pub.publish(procImage.toImageMsg());
    } else {
        ROS_INFO("No subscribers.  Suppressing processed image.");
    }
    
    ObstacleArrayStamped msg;
    //fill in timestamp info TODO
    for (std::vector<Lane>::const_iterator it = le.getLanes().begin();
         it != le.getLanes().end();
         it++) {
        
        Obstacle obstacle;
        std::vector<geometry_msgs::Polygon> polys = buildPolygonsFromLane(*it);
        obstacle.type = Obstacle::TYPE_LANE;
        obstacle.polygons.insert(obstacle.polygons.end(), polys.begin(), polys.end());
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
   
    procImage_pub = it.advertise("image_processed", 1);
    image_transport::Subscriber sub = it.subscribe("image", 1, imageCallback);
    
    lanePoly_pub  = nh.advertise<ObstacleArrayStamped>("obstacles", 100);
    
    ros::spin();
}
