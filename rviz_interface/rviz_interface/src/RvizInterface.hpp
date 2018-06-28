#ifndef RVIZINTERFACE_HPP
#define RVIZINTERFACE_HPP

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>

#include "InteractiveObject.hpp"

#include <rviz_interface/InterfaceConfig.h>

class RvizInterface
{
protected:
	ros::NodeHandle _n; 
	ros::Publisher _objective_pub;
	ros::Publisher _visualization_pub;
	ros::Subscriber _config_sub;
	
	interactive_markers::InteractiveMarkerServer _server;

	std::vector<InteractiveObject*> _objects;

public: 
	RvizInterface();
	~RvizInterface();

	void configCallback(const rviz_interface::InterfaceConfig & new_config);
};

#endif