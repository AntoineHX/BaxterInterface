/*
 * ROS wrapper for the ASIFT_matcher object.
 * Track an object described in the references in a RGBD stream and publish it's center.
 * @author : antoine.harle@etu.upmc.Fr
 * @see : ASIFT_matcher.cpp/.hpp, asift_match.launch
 */

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

	message_filters::Synchronizer<MySyncPolicy>* Timesync;
	
	//Matcher
	int _num_tilt; //Number of tilts
	float _filter_coeff; //Filter parameter
	ASIFT_matcher matcher; //Matcher

	MATCHER_STATUS _status; //Matcher status

	std::string tracked_object; //Name of the tracked object.

public:
	ROS_matcher();
	~ROS_matcher();
	void cameraCallback(const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
	
};
#endif