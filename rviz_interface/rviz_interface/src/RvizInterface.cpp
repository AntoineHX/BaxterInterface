#include "RvizInterface.hpp"

RvizInterface::RvizInterface(): _server("RvizInterface")
{
	//Topic you want to publish
	_objective_pub = _n.advertise<rviz_interface::StateSpace>("/RvizInterface/state_objective", 10);

	//Topic you want to subscribe
	_config_sub = _n.subscribe("/RvizInterface/interface_config", 1, &RvizInterface::configCallback, this);

	_objects.push_back(new InteractiveObject(&_objective_pub, &_server, "Test", (int) rviz_interface::StateSpace::STATE_3D, (int) visualization_msgs::Marker::CUBE, tf::Vector3(0,0,0)));
	// _objects[0].addButtonControl();
	_objects[0]->add6DOFcontrol();

	_objects.push_back(new InteractiveObject(&_objective_pub, &_server, "Test 2", (int) rviz_interface::StateSpace::STATE_2D, (int) visualization_msgs::Marker::CUBE, tf::Vector3(3,3,0)));
	// _objects[0].addButtonControl();
	_objects[1]->add3DOFcontrol();
}

RvizInterface::~RvizInterface()
{
	for(unsigned int i=0;i<_objects.size();i++)
	{
		delete _objects[i];
	}
}

void RvizInterface::configCallback(const rviz_interface::InterfaceConfig & new_config)
{
	for(unsigned int i=0;i<_objects.size();i++)
	{
		_objects[i]->state().objective_type = new_config.objective_type;
		_objects[i]->state().max_error = new_config.max_error;
	}
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Rviz_Interface");

  RvizInterface Interface;

  ros::spin();

  return 0;
}