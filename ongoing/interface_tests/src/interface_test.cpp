#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>

#include <interface_tests/StateSpaceT.h>

using namespace visualization_msgs;

// class InteractiveObject
// {
// protected:
// 	unsigned int _objectID; //Identifiant de l'objet
// 	static unsigned int nextObjectID;
// 	std::string _name;
// 	unsigned int _type;
// 	//std::vector<visualization_msgs::InteractiveMarker> _int_markers;
// 	visualization_msgs::InteractiveMarker _int_marker;

// 	interactive_markers::InteractiveMarkerServer& _server;
// 	interface_tests::StateSpaceT _state;

// public:
// 	InteractiveObject(const std::string& name, unsigned int type, interactive_markers::InteractiveMarkerServer& server): _name(name), _type(type), _server(server)
// 	{
// 		_objectID = nextObjectID++;
// 		_state.state_type=type;
// 		//_server = server;
// 		//_int_marker = createInteractiveMarker(_name+_objectID,box_marker);
// 	}

// 	//std::vector<visualization_msgs::InteractiveMarker>& markers(){ return _int_markers;}
// 	visualization_msgs::InteractiveMarker& marker(){ return _int_marker;}
// 	interface_tests::StateSpaceT& state(){ return _state;}
// };

// unsigned int InteractiveObject::nextObjectID = 1;

class InterfaceTest
{
private :
  ros::NodeHandle _n; 
  ros::Publisher _objective_pub;
  //ros::Subscriber sub_;
  interactive_markers::InteractiveMarkerServer server;

  //std::vector<visualization_msgs::InteractiveMarker> _int_markers;
 // std::vector<InteractiveObject> _objects;

public:

  InterfaceTest(): server("interface_tests")
  {
	//Topic you want to publish
	_objective_pub = _n.advertise<interface_tests::StateSpaceT>("state_objective", 10);

	//Topic you want to subscribe
	//sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);


	// create a grey box marker
	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::CUBE;
	box_marker.scale.x = 0.45;
	box_marker.scale.y = 0.45;
	box_marker.scale.z = 0.45;
	box_marker.color.r = 0.5;
	box_marker.color.g = 0.5;
	box_marker.color.b = 0.5;
	box_marker.color.a = 1.0;

	tf::Vector3 position;
  	position = tf::Vector3(0, 0, 0);

  	visualization_msgs::InteractiveMarker int_box = createInteractiveMarker("6DOF 1",box_marker);
  	addButtoncontrol(int_box);
	add6DOFcontrol(int_box);


  	visualization_msgs::Marker sphere_marker;
	sphere_marker.type = visualization_msgs::Marker::SPHERE;
	sphere_marker.scale.x = 0.45;
	sphere_marker.scale.y = 0.45;
	sphere_marker.scale.z = 0.45;
	sphere_marker.color.r = 0.5;
	sphere_marker.color.g = 0.5;
	sphere_marker.color.b = 0.5;
	sphere_marker.color.a = 1.0;

  	position = tf::Vector3(-3, 3, 0);

	visualization_msgs::InteractiveMarker int_sphere = createInteractiveMarker("3DOF",sphere_marker,position);
  	addButtoncontrol(int_sphere);
	add3DOFcontrol(int_sphere);

	visualization_msgs::Marker cyl_marker;
	cyl_marker.type = visualization_msgs::Marker::CYLINDER;
	cyl_marker.scale.x = 0.45;
	cyl_marker.scale.y = 0.45;
	cyl_marker.scale.z = 0.45;
	cyl_marker.color.r = 0.5;
	cyl_marker.color.g = 0.5;
	cyl_marker.color.b = 0.5;
	cyl_marker.color.a = 1.0;

  	position = tf::Vector3(3, 3, 0);

	visualization_msgs::InteractiveMarker int_cyl = createInteractiveMarker("Button",cyl_marker, position);
  	addButtoncontrol(int_cyl);
  }

   //Name must be unique
  visualization_msgs::InteractiveMarker createInteractiveMarker(const std::string& name, visualization_msgs::Marker& marker, const tf::Vector3& position = tf::Vector3(0,0,0))
  {
  	//// Création d'un marker interactif ////
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "base_link";
	int_marker.header.stamp=ros::Time::now();
	int_marker.name = name;
	int_marker.description = name;
	tf::pointTFToMsg(position, int_marker.pose.position);

	// create a non-interactive control which contains the box
	visualization_msgs::InteractiveMarkerControl container;
	container.always_visible = true; //Toujours visible hors du mode interaction
	container.markers.push_back( marker );

	// add the control to the interactive marker
	int_marker.controls.push_back( container );

	// _int_markers.push_back(int_marker);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	//server.insert(_int_marker, &InterfaceTest::processFeedback);
	server.insert(int_marker, boost::bind(&InterfaceTest::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	server.applyChanges();

	return int_marker;
  }

  void addButtoncontrol(visualization_msgs::InteractiveMarker& int_marker)
  {
  	//// Ajout d'interactions ////
	visualization_msgs::InteractiveMarkerControl marker_control;

	int_marker.controls[0].interaction_mode = InteractiveMarkerControl::BUTTON;
	int_marker.controls[0].name= "Button";

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	//server.insert(_int_marker, &InterfaceTest::processFeedback);
	server.insert(int_marker, boost::bind(&InterfaceTest::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	server.applyChanges();
  }

  //6DOF + Boutton
  //Name must be unique
  void add6DOFcontrol(visualization_msgs::InteractiveMarker& int_marker)
  {
	//// Ajout d'interactions ////
	// create a control which will move the box
	// this control does not contain any markers,
	// which will cause RViz to insert two arrows
	visualization_msgs::InteractiveMarkerControl marker_control;

	//_int_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D; //Deplacement par "grab" de la souris (CTRL + Souris = rotation)
	//int_marker.controls[0].interaction_mode = InteractiveMarkerControl::BUTTON; //Ajout fonctionalité Boutton (Validation ?)
	int_marker.controls[0].name= "3D";

	marker_control.orientation_mode = InteractiveMarkerControl::FIXED;
	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 1;
	marker_control.orientation.y = 0;
	marker_control.orientation.z = 0;
	//Ajout des controles associées
	marker_control.name = "rotate_x";
	marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(marker_control);
	marker_control.name = "move_x";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(marker_control);

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 0;
	marker_control.orientation.y = 1;
	marker_control.orientation.z = 0;
	//Ajout des controles associées
	marker_control.name = "rotate_y";
	marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(marker_control);
	marker_control.name = "move_y";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(marker_control);

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 0;
	marker_control.orientation.y = 0;
	marker_control.orientation.z = 1;
	//Ajout des controles associées
	marker_control.name = "rotate_z";
	marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(marker_control);
	marker_control.name = "move_z";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(marker_control);

	// _int_markers.push_back(int_marker);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	//server.insert(_int_marker, &InterfaceTest::processFeedback);
	server.insert(int_marker, boost::bind(&InterfaceTest::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	server.applyChanges();
  }

  //3DOF + Boutton
   //Name must be unique
  void add3DOFcontrol(visualization_msgs::InteractiveMarker& int_marker)
  {
	//// Ajout d'interactions ////
	// create a control which will move the box
	// this control does not contain any markers,
	// which will cause RViz to insert two arrows
	visualization_msgs::InteractiveMarkerControl marker_control;

	//_int_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D; //Deplacement par "grab" de la souris (CTRL + Souris = rotation)
	// int_marker.controls[0].interaction_mode = InteractiveMarkerControl::BUTTON; //Ajout fonctionalité Boutton (Validation ?)
	int_marker.controls[0].name= "2D";

	marker_control.orientation_mode = InteractiveMarkerControl::FIXED;
	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 1;
	marker_control.orientation.y = 0;
	marker_control.orientation.z = 0;
	//Ajout des controles associées
	marker_control.name = "move_x";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(marker_control);

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 0;
	marker_control.orientation.y = 1;
	marker_control.orientation.z = 0;
	//Ajout des controles associées
	marker_control.name = "rotate_y";
	marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(marker_control);

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 0;
	marker_control.orientation.y = 0;
	marker_control.orientation.z = 1;
	//Ajout des controles associées
	marker_control.name = "move_z";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(marker_control);

	// _int_markers.push_back(int_marker);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	//server.insert(_int_marker, &InterfaceTest::processFeedback);
	server.insert(int_marker, boost::bind(&InterfaceTest::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	server.applyChanges();
  }

  	//Faire deffedback different pour chaque type ?
	void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
	{
		//// SEND OBJECTIVE ////
		if(feedback->event_type == InteractiveMarkerFeedback::BUTTON_CLICK)
		{
			interface_tests::StateSpaceT obj;
			obj.name = feedback->marker_name +" objective - "+ feedback->control_name;

			if(feedback->control_name == "Button")
			{
				obj.state_type = 3; //interface_tests::STATE_BOOL;

				obj.discrete_data.push_back(true);
			}
			else if(feedback->control_name == "2D")
			{
				obj.state_type = 1; //interface_tests::STATE_2D;

				obj.real_data.push_back(feedback->pose.position.x);
				obj.real_data.push_back(feedback->pose.position.y);
				// obj.real_data.push_back(feedback->pose.position.z);
				// obj.real_data.push_back(feedback->pose.orientation.w);
				// obj.real_data.push_back(feedback->pose.orientation.x);
				// obj.real_data.push_back(feedback->pose.orientation.y);
				obj.real_data.push_back(feedback->pose.orientation.z);
			}
			else if(feedback->control_name == "3D")
			{
				obj.state_type = 2; //interface_tests::STATE_3D;

				obj.real_data.push_back(feedback->pose.position.x);
				obj.real_data.push_back(feedback->pose.position.y);
				obj.real_data.push_back(feedback->pose.position.z);
				obj.real_data.push_back(feedback->pose.orientation.w);
				obj.real_data.push_back(feedback->pose.orientation.x);
				obj.real_data.push_back(feedback->pose.orientation.y);
				obj.real_data.push_back(feedback->pose.orientation.z);
			}
			else
			{
				obj.state_type = 0; // interface_tests::ORDINARY;

				obj.real_data.push_back(feedback->pose.position.x);
				obj.real_data.push_back(feedback->pose.position.y);
				obj.real_data.push_back(feedback->pose.position.z);
				obj.real_data.push_back(feedback->pose.orientation.w);
				obj.real_data.push_back(feedback->pose.orientation.x);
				obj.real_data.push_back(feedback->pose.orientation.y);
				obj.real_data.push_back(feedback->pose.orientation.z);
			}
			//obj.discrete_data.push_back(1);
			//obj.discrete_data.push_back(false);

			//obj.dimension = obj.real_data.size() + obj.discrete_data.size();

			obj.description="Test du message StateSpaceT !";

			_objective_pub.publish(obj);
		}

		//// PRINT INFO ////
	 //  ROS_INFO_STREAM( feedback->marker_name << " is now at "
	 //      << feedback->pose.position.x << ", " << feedback->pose.position.y
	 //      << ", " << feedback->pose.position.z );

		// std::ostringstream s;
	 //  	s << "Feedback from marker '" << feedback->marker_name << "' "<< " / control '" << feedback->control_name << "'";

	 //  	std::ostringstream mouse_point_ss;
		// if( feedback->mouse_point_valid )
		// {
		//   	mouse_point_ss << " at " << feedback->mouse_point.x
		//                    << ", " << feedback->mouse_point.y
		//                    << ", " << feedback->mouse_point.z
		//                    << " in frame " << feedback->header.frame_id;
		// }
		
		// switch ( feedback->event_type )
		// {
		//    	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		//     ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
		//     break;

		//     case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
	 //      ROS_INFO_STREAM( s.str() << ": pose changed"
	 //          << "\nposition = "
	 //          << feedback->pose.position.x
	 //          << ", " << feedback->pose.position.y
	 //          << ", " << feedback->pose.position.z
	 //          << "\norientation = "
	 //          << feedback->pose.orientation.w
	 //          << ", " << feedback->pose.orientation.x
	 //          << ", " << feedback->pose.orientation.y
	 //          << ", " << feedback->pose.orientation.z
	 //          << "\nframe: " << feedback->header.frame_id
	 //          << " time: " << feedback->header.stamp.sec << "sec, "
	 //          << feedback->header.stamp.nsec << " nsec" );
	 //      break;

	 //    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
	 //      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
	 //      break;

	 //    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
	 //      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
	 //      break;
		// }
	}

	// Marker makeBox( InteractiveMarker &msg )
	// {
	//   Marker marker;

	//   marker.type = Marker::CUBE;
	//   marker.scale.x = msg.scale * 0.45;
	//   marker.scale.y = msg.scale * 0.45;
	//   marker.scale.z = msg.scale * 0.45;
	//   marker.color.r = 0.5;
	//   marker.color.g = 0.5;
	//   marker.color.b = 0.5;
	//   marker.color.a = 1.0;

	//   return marker;
	// }

	// InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
	// {
	//   InteractiveMarkerControl control;
	//   control.always_visible = true;
	//   control.markers.push_back( makeBox(msg) );
	//   msg.controls.push_back( control );

	//   return msg.controls.back();
	// }

	//const std::vector<visualization_msgs::InteractiveMarker>& markers() const { return _int_markers;}
	// std::vector<visualization_msgs::InteractiveMarker>& markers() { return _int_markers;}
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "interface_tests");

  InterfaceTest Interface;

  ros::spin();

  return 0;
}