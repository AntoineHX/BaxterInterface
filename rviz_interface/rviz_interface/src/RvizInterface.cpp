#include "RvizInterface.hpp"

//Constructeur
RvizInterface::RvizInterface(): _server("RvizInterface")
{
	//Topic you want to publish
	_objective_pub = _n.advertise<rviz_interface::StateSpace>("/RvizInterface/state_objective", 10);
	_visualization_pub = _n.advertise<visualization_msgs::Marker>("/RvizInterface/visual_marker", 10);

	//Topic you want to subscribe
	_config_sub = _n.subscribe("/RvizInterface/interface_config", 1, &RvizInterface::configCallback, this);
	_position_sub = _n.subscribe("/object_center", 1, &RvizInterface::positionCallback, this);

	_objects.push_back(new InteractiveObject(&_server, "6DOF", (int) rviz_interface::StateSpace::STATE_3D, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(0,0,0)));
	_objects[0]->add6DOFcontrol();

	_objects.push_back(new InteractiveObject(&_server, "3DOF", (int) rviz_interface::StateSpace::STATE_2D, (int) visualization_msgs::Marker::SPHERE, tf::Vector3(1,1,0)));
	_objects[1]->add3DOFcontrol();


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

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "Rviz_Interface");

  RvizInterface Interface;

  ros::spin();

  return 0;
}