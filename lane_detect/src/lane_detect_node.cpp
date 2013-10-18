#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped	>
#include <sensor_msgs/Image.h>
#include <core_msgs/Obstacle.h>

#include <math.h>

ros::Publisher lanePoly_pub, projImage_pub;

void imageCallback(const sensor_msgs::Image::ConstPtr& image) {
}

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "lane_detect");
  
  ros::NodeHandle n;

 
  lanePoly_pub  = n.advertise<geometry_msgs::PolygonStamped>("PhidgetMotor", 100);
  projImage_pub = n.advertise<sensor_msgs::Image>("image_projection", 100);
  ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("image", 1000, imageCallback);
  ros::spin();
}
