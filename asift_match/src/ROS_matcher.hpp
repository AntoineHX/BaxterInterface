#ifndef ROSMATCHER_HPP
#define ROSMATCHER_HPP

#include <ros/ros.h>
#include <tf/tf.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

#include "ASIFT_matcher.hpp"

enum MATCHER_STATUS{
	MATCHER_STATUS_IDLE=0,
	MATCHER_STATUS_PROCESSING,
	MATCHER_STATUS_WAITING_INIT};

class ROS_matcher
{
protected:
	ros::NodeHandle _nh; 

	//Publisher ROS
	ros::Publisher _center_pub;

	//Subscriber ROS
	// ros::Subscriber _image_sub;
	
	//Matcher
	int _num_tilt;
	ASIFT_matcher matcher;

	MATCHER_STATUS _status;

public:
	ROS_matcher();
	// ~ROS_matcher();
	void cameraCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
	
};
#endif