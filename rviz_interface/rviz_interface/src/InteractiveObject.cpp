#include "InteractiveObject.hpp"

unsigned int InteractiveObject::nextObjectID = 1;

InteractiveObject::InteractiveObject(interactive_markers::InteractiveMarkerServer* server, const std::string& name, unsigned int type, unsigned int shape = Marker::CUBE, const tf::Vector3& position = tf::Vector3(0,0,0)) : _name(name), _type(type), _server(server), _showVisuals(true), _followObject(true)
{
	_objectID = nextObjectID++;
	_state.name = name;
	_state.state_type=type;
	_state.max_error = 0; //Default error

	Marker marker;
	marker.type = shape;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;

	createInteractiveMarker(marker, position);
	addButtoncontrol();
}

 //Name must be unique
void InteractiveObject::createInteractiveMarker(Marker& marker, const tf::Vector3& position = tf::Vector3(0,0,0))
{
	//// Création d'un marker interactif ////
	_int_marker.header.frame_id = "/map";
	_int_marker.header.stamp=ros::Time::now();
	_int_marker.name = _name; //ATTENTION !
	_int_marker.description = _name;
	tf::pointTFToMsg(position, _int_marker.pose.position);
	_int_marker.scale = (marker.scale.x + marker.scale.y + marker.scale.z)/3 ;

	// create a non-interactive control which contains the box
	InteractiveMarkerControl interactive_container;
	interactive_container.name = "control";
	interactive_container.always_visible = true; //Toujours visible hors du mode interaction
	interactive_container.markers.push_back( marker );

	// add the control to the interactive marker
	_int_marker.controls.push_back( interactive_container );

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	_server->insert(_int_marker, boost::bind(&InteractiveObject::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	_server->applyChanges();
}

void InteractiveObject::processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback )
{
	_followObject = false; //Objet manipulé -> on les libèrent

	//// Update Visual markers ////
	// if(_showVisuals)
	// {
	// 	//Màj des visuels
	// 	for(unsigned int i=0; i<visual_container.markers.size();i++)
	// 	{
	// 		visual_container.markers[i].pose = feedback->pose;
	// 	}
	// 	//Ajout du container
	// 	ROS_INFO_STREAM("Name : "<<_int_marker.controls.back().name<<" / Size : "<<_int_marker.controls.size());
	// 	if(_int_marker.controls.back().name=="visual") 
	// 	{
	// 		_int_marker.controls.back()=visual_container; //Modification du container deja present
	// 	}
	// 	else
	// 	{
	// 		_int_marker.controls.push_back( visual_container ); //Ajout
	// 	}
	// }

	if(feedback->event_type == InteractiveMarkerFeedback::BUTTON_CLICK )
	{
		// Send objective
		if(_objective_pub && feedback->control_name == "control")
		{
			rviz_interface::StateSpace msg;
			msg.name = _state.name;
			msg.state_type = _state.state_type;
			msg.objective_type = _state.objective_type;
			msg.max_error = _state.max_error;

			//Ajout des informations de position
			msg.real_data.push_back(feedback->pose.position.x);
			msg.real_data.push_back(feedback->pose.position.y);
			msg.real_data.push_back(feedback->pose.position.z);
			msg.real_data.push_back(feedback->pose.orientation.w);
			msg.real_data.push_back(feedback->pose.orientation.x);
			msg.real_data.push_back(feedback->pose.orientation.y);
			msg.real_data.push_back(feedback->pose.orientation.z);

			//Ajout des inforamtions supplémentaires
			for(unsigned int i=0;i<_state.real_data.size();i++)
			{
				msg.real_data.push_back(_state.real_data[i]);
			}
			for(unsigned int i=0;i<_state.discrete_data.size();i++)
			{
				msg.discrete_data.push_back(_state.discrete_data[i]);
			}

			//Publication de l'objectif
			_objective_pub->publish(msg);
		}
		// Hide visuals
		// if(feedback->control_name == "visual")
		// {
		// 	_int_marker.controls.pop_back();
		// 	_server->insert(_int_marker);
		// 	_server->applyChanges();
		// 	_showVisuals = false;
		// }
	}	
}

//Attention a faire en dernier ! visual container doit etre le dernier control
void InteractiveObject::addVisuals()
{
	//Create the error area
	// InteractiveMarkerControl visual_container;
	visual_container.name = "visual";
	visual_container.interaction_mode = InteractiveMarkerControl::BUTTON; //Click on visual effect to make them disappear

	Marker error_marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
    // error_marker.header.frame_id = "/map";
    // error_marker.header.stamp = ros::Time::now();

    // // Set the namespace and id for this marker.  This serves to create a unique ID
    // // Any marker sent with the same namespace and id will overwrite the old one
    // error_marker.ns = _name;
    // error_marker.id = 0;
	error_marker.type = visualization_msgs::Marker::SPHERE;
	error_marker.scale.x = 1;
	error_marker.scale.y = 1;
	error_marker.scale.z = 1;
	error_marker.color.r = 0.5;
	error_marker.color.g = 0.5;
	error_marker.color.b = 0.5;
	error_marker.color.a = 0.5;
	error_marker.pose = _int_marker.pose;

	// _visual_markers.push_back(error_marker);

	visual_container.markers.push_back( error_marker );

	_int_marker.controls.push_back( visual_container );

	_server->insert(_int_marker);
	_server->applyChanges();
}

void InteractiveObject::addButtoncontrol()
{
	//// Ajout d'interactions ////
	InteractiveMarkerControl marker_control;

	_int_marker.controls[0].interaction_mode = InteractiveMarkerControl::BUTTON;
	// _int_marker.controls[0].name= "Button";

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	_server->insert(_int_marker);

	// 'commit' changes and send to all clients
	_server->applyChanges();
}

//6DOF + Boutton
void InteractiveObject::add6DOFcontrol()
{
	//// Ajout d'interactions ////
	InteractiveMarkerControl marker_control;

	//_int_marker.controls[0].name= "3D";

	marker_control.orientation_mode = InteractiveMarkerControl::FIXED;
	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 1;
	marker_control.orientation.y = 0;
	marker_control.orientation.z = 0;
	//Ajout des controles associées
	marker_control.name = "rotate_x";
	marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	_int_marker.controls.push_back(marker_control);
	marker_control.name = "move_x";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	_int_marker.controls.push_back(marker_control);

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 0;
	marker_control.orientation.y = 1;
	marker_control.orientation.z = 0;
	//Ajout des controles associées
	marker_control.name = "rotate_y";
	marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	_int_marker.controls.push_back(marker_control);
	marker_control.name = "move_y";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	_int_marker.controls.push_back(marker_control);

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 0;
	marker_control.orientation.y = 0;
	marker_control.orientation.z = 1;
	//Ajout des controles associées
	marker_control.name = "rotate_z";
	marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	_int_marker.controls.push_back(marker_control);
	marker_control.name = "move_z";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	_int_marker.controls.push_back(marker_control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	_server->insert(_int_marker);

	// 'commit' changes and send to all clients
	_server->applyChanges();
}

//3DOF + Boutton
void InteractiveObject::add3DOFcontrol()
{
	//// Ajout d'interactions ////
	InteractiveMarkerControl marker_control;

	//_int_marker.controls[0].name= "2D";

	marker_control.orientation_mode = InteractiveMarkerControl::FIXED;
	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 1;
	marker_control.orientation.y = 0;
	marker_control.orientation.z = 0;
	//Ajout des controles associées
	marker_control.name = "move_x";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	_int_marker.controls.push_back(marker_control);

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 0;
	marker_control.orientation.y = 1;
	marker_control.orientation.z = 0;
	//Ajout des controles associées
	marker_control.name = "rotate_y";
	marker_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	_int_marker.controls.push_back(marker_control);

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = 0;
	marker_control.orientation.y = 0;
	marker_control.orientation.z = 1;
	//Ajout des controles associées
	marker_control.name = "move_z";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	_int_marker.controls.push_back(marker_control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	_server->insert(_int_marker);

	// 'commit' changes and send to all clients
	_server->applyChanges();
}

void InteractiveObject::setErrorArea(double error)
{
	_showVisuals = true;

	_state.max_error = error;
}

void InteractiveObject::moveTo(const tf::Vector3& new_pos)
{
	tf::pointTFToMsg(new_pos, _int_marker.pose.position);
	_server->insert(_int_marker);
	_server->applyChanges();
}