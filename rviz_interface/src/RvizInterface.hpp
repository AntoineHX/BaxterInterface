/*
 * Rviz interface to send objective.
 * Composed of 3D marker and a configuration panel.
 * @author : antoine.harle@etu.upmc.Fr
 * @see : InteractiveObject.cpp/.hpp, InterfacePanel.cpp/.hpp
 */

#ifndef RVIZINTERFACE_HPP
#define RVIZINTERFACE_HPP

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>

#include "InteractiveObject.hpp"

#include <rviz_interface/InterfaceConfig.h>

#include <rviz_interface/NamedPoint.h>

class RvizInterface
{
protected:
	ros::NodeHandle _n; 

	//Publisher ROS
	ros::Publisher _objective_pub;
	ros::Publisher _visualization_pub;

	//Subscriber ROS
	ros::Subscriber _config_sub;
	ros::Subscriber _position_sub;
	
	//Serveur de marker interactif
	interactive_markers::InteractiveMarkerServer _server;

	//Objets de l'interface
	std::vector<InteractiveObject*> _objects;

public: 
	RvizInterface(const std::string & server_topic);
	~RvizInterface();

	//Fonction Callback du panel Rviz gérant les configurations
	void configCallback(const rviz_interface::InterfaceConfig & new_config);

	//Fonction callback gérant la position des objets
	void positionCallback(const rviz_interface::NamedPoint & new_center);
};

#endif