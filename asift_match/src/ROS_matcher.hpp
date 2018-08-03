#ifndef ROSMATCHER_HPP
#define ROSMATCHER_HPP

#include <ros/ros.h>
#include <tf/tf.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <rviz_interface/NamedPoint.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/impl/point_types.hpp>

#include "ASIFT_matcher.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

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
	// message_filters::Subscriber<sensor_msgs::CameraInfo>* info_sub;
	message_filters::Subscriber<sensor_msgs::Image>* _image_sub;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* _pointcloud_sub;
	
	//Matcher
	int _num_tilt, _filter_coeff;
	ASIFT_matcher matcher;

	MATCHER_STATUS _status;

	message_filters::Synchronizer<MySyncPolicy>* Timesync;

	std::string tracked_object;

public:
	ROS_matcher();
	~ROS_matcher();
	void cameraCallback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
	
};
#endif