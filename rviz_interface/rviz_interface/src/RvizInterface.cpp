/*
 * Rviz interface to send objective.
 * Composed of 3D marker and a configuration panel.
 * @author : antoine.harle@etu.upmc.Fr
 * @see : InteractiveObject.cpp/.hpp, InterfacePanel.cpp/.hpp
 */

#include "RvizInterface.hpp"

//Constructeur
RvizInterface::RvizInterface(): _server("RvizInterface")
{
	std::string objective_topic, vizualization_topic, config_topic, position_topic;
	std::vector<std::string> object_names;
	std::vector<int> object_types;

	//Load Topics param
	_n.param<std::string>("objective_topic", objective_topic,"/RvizInterface/state_objective");
	_n.param<std::string>("vizualization_topic", vizualization_topic,"/RvizInterface/visual_marker");
	_n.param<std::string>("config_topic", config_topic,"/RvizInterface/interface_config");
	_n.param<std::string>("object_center_topic", position_topic,"/RvizInterface/object_center");

	//Topic you want to publish
	_objective_pub = _n.advertise<rviz_interface::StateSpace>(objective_topic, 10);
	_visualization_pub = _n.advertise<visualization_msgs::Marker>(vizualization_topic, 10);

	//Topic you want to subscribe
	_config_sub = _n.subscribe(config_topic, 1, &RvizInterface::configCallback, this);
	_position_sub = _n.subscribe(position_topic, 1, &RvizInterface::positionCallback, this);

	//Load objects
	_n.getParam("tracked_object_names",object_names);
	_n.getParam("tracked_object_types",object_types);

	int names_size = object_names.size();

	if(names_size<object_types.size())
	{
		ROS_WARN("Missing object names. Objects without names won't be added...");
	}
	else if(names_size>object_types.size())
	{
		ROS_WARN("Missing object types. Missing types are set to Ordinary by default...");
		while(names_size>object_types.size())
		{
			object_types.push_back(0);
		}
	}

	for(unsigned int i=0; i< names_size;i++)
	{
		switch(object_types[i])
		{
			case 0:
				_objects.push_back(new InteractiveObject(&_server, object_names[i], (int) rviz_interface::StateSpace::ORDINARY, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(0,0,0)));
				_objects[i]->add6DOFcontrol();
				break;
			case 1:
				_objects.push_back(new InteractiveObject(&_server, object_names[i], (int) rviz_interface::StateSpace::STATE_BOOL, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(0,0,0)));
				_objects[i]->addButtoncontrol();
				break;
			case 2:
				_objects.push_back(new InteractiveObject(&_server, object_names[i], (int) rviz_interface::StateSpace::STATE_2D, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(0,0,0)));
				_objects[i]->add3DOFcontrol();
				break;
			case 3:
				_objects.push_back(new InteractiveObject(&_server, object_names[i], (int) rviz_interface::StateSpace::STATE_3D, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(0,0,0)));
				_objects[i]->add6DOFcontrol();
				break;
		}
	}
	// _objects.push_back(new InteractiveObject(&_server, object_name, (int) rviz_interface::StateSpace::STATE_3D, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(0,0,0)));
	// _objects[0]->add6DOFcontrol();

	// _objects.push_back(new InteractiveObject(&_server, "3DOF", (int) rviz_interface::StateSpace::STATE_2D, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(0,0,0)));
	// _objects[1]->add3DOFcontrol();

	// _objects.push_back(new InteractiveObject(&_server, "3DOF 2", (int) rviz_interface::StateSpace::STATE_2D, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(-1,1,0)));
	// _objects[2]->add3DOFcontrol(tf::Vector3(0.5,0.5,0));

	// _objects.push_back(new InteractiveObject(&_server, "Button", (int) rviz_interface::StateSpace::STATE_3D, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(1,-1,0)));

	for(unsigned int i=0;i<_objects.size();i++)
	{
		_objects[i]->setObjectivePublisher(&_objective_pub);
		_objects[i]->setVisualizationPublisher(&_visualization_pub);
		_objects[i]->addVisuals();

		//Synchro des frames
		// _objects[i]->int_marker().header.frame_id = "/camera_rgb_optical_frame";
		// _objects[i]->err_marker().header.frame_id = "/camera_rgb_optical_frame";
	}
}

//Destructeur
RvizInterface::~RvizInterface()
{
	for(unsigned int i=0;i<_objects.size();i++)
	{
		delete _objects[i];
	}
}

//Fonction Callback du panel Rviz gérant les configurations
void RvizInterface::configCallback(const rviz_interface::InterfaceConfig & new_config)
{
	for(unsigned int i=0;i<_objects.size();i++)
	{
		_objects[i]->state().objective_type = new_config.objective_type;
		// _objects[i]->state().max_error = new_config.max_error;
		_objects[i]->setErrorArea(new_config.max_error);
		_objects[i]->showVisuals(new_config.show_visuals);
		if(new_config.follow_object && !_objects[i]->isFollowing()) //Signal reset + pas de suivie
		{
			_objects[i]->follow(); //Le marker suit de nouveau l'objet
		}
	}
}

//Fonction callback gérant la position de l'objet
//PROVISOIRE : Pour un seul objet avec test PCL
// void RvizInterface::positionCallback(const geometry_msgs::PointStamped & new_center)
// {
// 	//Synchro des frames
// 	_objects[0]->int_marker().header.frame_id = new_center.header.frame_id;
// 	_objects[0]->err_marker().header.frame_id = new_center.header.frame_id;

// 	if(_objects[0]->isFollowing()) //Le marqueur suit l'objet
// 	{
// 		//Deplacement du marqueur
// 		tf::Vector3 position = tf::Vector3(new_center.point.x,new_center.point.y,new_center.point.z);
// 		_objects[0]->moveTo(position);
// 	}
// }

//Fonction callback gérant la position des objets
void RvizInterface::positionCallback(const rviz_interface::NamedPoint & new_center)
{
	for(unsigned int i=0;i<_objects.size();i++)
	{
		if(_objects[i]->name()==new_center.name)
		{
			//Synchro des frames
			// _objects[i]->int_marker().header.frame_id = new_center.header.frame_id;
			// _objects[i]->err_marker().header.frame_id = new_center.header.frame_id;

			if(_objects[i]->isFollowing()) //Le marqueur suit l'objet
			{
				//Deplacement du marqueur
				tf::Vector3 position = tf::Vector3(new_center.point.x,new_center.point.y,new_center.point.z);
				_objects[i]->moveTo(position);
			}
			break;
		}
	}
}

//RvizInterface node.
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Rviz_Interface");

  RvizInterface Interface;

  ros::spin();

  return 0;
}