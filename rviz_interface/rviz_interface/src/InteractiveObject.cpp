#include "InteractiveObject.hpp"

unsigned int InteractiveObject::nextObjectID = 1;

InteractiveObject::InteractiveObject(ros::Publisher* objective_pub, interactive_markers::InteractiveMarkerServer* server, const std::string& name, unsigned int type, unsigned int shape = Marker::CUBE, const tf::Vector3& position = tf::Vector3(0,0,0)) : _name(name), _type(type), _server(server), _objective_pub(objective_pub)
{
	_objectID = nextObjectID++;
	_state.state_type=type;
	_state.max_error = 0; //Default error

	Marker marker;
	marker.type = shape;
	marker.scale.x = 0.45;
	marker.scale.y = 0.45;
	marker.scale.z = 0.45;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;

	createInteractiveMarker(marker, position);
	addButtoncontrol();
	// add6DOFcontrol();
}

 //Name must be unique
void InteractiveObject::createInteractiveMarker(Marker& marker, const tf::Vector3& position = tf::Vector3(0,0,0))
{
	//// Création d'un marker interactif ////
	_int_marker.header.frame_id = "map";
	_int_marker.header.stamp=ros::Time::now();
	_int_marker.name = _name; //ATTENTION !
	_int_marker.description = _name;
	tf::pointTFToMsg(position, _int_marker.pose.position);

	// create a non-interactive control which contains the box
	InteractiveMarkerControl container;
	container.always_visible = true; //Toujours visible hors du mode interaction
	container.markers.push_back( marker );

	// add the control to the interactive marker
	_int_marker.controls.push_back( container );

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	_server->insert(_int_marker, boost::bind(&InteractiveObject::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	_server->applyChanges();
}

void InteractiveObject::processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback )
	{
	//// SEND OBJECTIVE ////
	if(feedback->event_type == InteractiveMarkerFeedback::BUTTON_CLICK)
	{
		_state.name = _name;

		//Ajout des informations de position
		_state.real_data.push_back(feedback->pose.position.x);
		_state.real_data.push_back(feedback->pose.position.y);
		_state.real_data.push_back(feedback->pose.position.z);
		_state.real_data.push_back(feedback->pose.orientation.w);
		_state.real_data.push_back(feedback->pose.orientation.x);
		_state.real_data.push_back(feedback->pose.orientation.y);
		_state.real_data.push_back(feedback->pose.orientation.z);

		//Publication de l'objectif
		_objective_pub->publish(_state);

		//Problème d'ajout continue de data
		_state.real_data.clear(); //Attention peut poser problème pour ajouter des infos supplémentaires (nécessité de les rajouter à chaque feedback)
	}
}

void InteractiveObject::addButtoncontrol()
{
	//// Ajout d'interactions ////
	InteractiveMarkerControl marker_control;

	_int_marker.controls[0].interaction_mode = InteractiveMarkerControl::BUTTON;
	// _int_marker.controls[0].name= "Button";

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	_server->insert(_int_marker, boost::bind(&InteractiveObject::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	_server->applyChanges();
}

//6DOF + Boutton
void InteractiveObject::add6DOFcontrol()
{
	//// Ajout d'interactions ////
	InteractiveMarkerControl marker_control;

	_int_marker.controls[0].name= "3D";

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
	_server->insert(_int_marker, boost::bind(&InteractiveObject::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	_server->applyChanges();
}

//3DOF + Boutton
void InteractiveObject::add3DOFcontrol()
{
	//// Ajout d'interactions ////
	InteractiveMarkerControl marker_control;

	_int_marker.controls[0].name= "2D";

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
	_server->insert(_int_marker, boost::bind(&InteractiveObject::processFeedback,this,_1)); //Boost bind -> passage en argument de la fonction membre de interface_test

	// 'commit' changes and send to all clients
	_server->applyChanges();
}