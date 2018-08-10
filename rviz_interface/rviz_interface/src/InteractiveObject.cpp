/*
 * Rviz interactive object.
 * @author : antoine.harle@etu.upmc.Fr
 * @see : RvizInterface.cpp/.hpp
 */

#include "InteractiveObject.hpp"

unsigned int InteractiveObject::nextObjectID = 1;

//Constructeur
InteractiveObject::InteractiveObject(interactive_markers::InteractiveMarkerServer* server, const std::string& name, unsigned int type, unsigned int shape = Marker::CUBE, const tf::Vector3& position) : _name(name), _type(type), _server(server), _showVisuals(true), _followObject(true)
{
	_objectID = nextObjectID++;
	_state.name = name;
	_state.state_type=type;
	_state.max_error = 0; //Default error

	Marker marker;
	marker.type = shape;
	marker.scale.x = 0.07;
	marker.scale.y = 0.07;
	marker.scale.z = 0.07;
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	marker.color.a = 1.0;

	createInteractiveMarker(marker, position);
	addButtoncontrol();
}

 //Construit un InteractiveMarker
void InteractiveObject::createInteractiveMarker(Marker& marker, const tf::Vector3& position)
{
	//// Création d'un marker interactif ////
	// _int_marker.header.frame_id = "/map"; //Par défaut
	_int_marker.header.frame_id = "/camera_rgb_optical_frame";
	_int_marker.header.stamp= ros::Time::now();
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

//Fonction callback du serveur d' InteractiveMarker
void InteractiveObject::processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback )
{
	_followObject = false; //Objet manipulé -> on les libèrent

	_int_marker.pose = feedback->pose; //Màj du marqueur interne

	if(feedback->event_type == InteractiveMarkerFeedback::BUTTON_CLICK )
	{
		// Send objective
		if(ros::ok() && _objective_pub && feedback->control_name == "control")
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
	}	
	updateVisuals();
}

//Ajoute une fonction bouton
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

//Ajoute des controle pour 6DOF (Déplacement dans l'espace)
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

//Ajoute des controle pour 3DOF (Déplacement dans le plan x/z)
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

//Ajoute des controle pour 3DOF (Déplacement dans le plan)
void InteractiveObject::add3DOFcontrol(const tf::Vector3& normal)
{
	//// Ajout d'interactions ////
	InteractiveMarkerControl marker_control;

	//_int_marker.controls[0].name= "2D";

	marker_control.orientation_mode = InteractiveMarkerControl::FIXED;

	//Orientation du vecteur
	marker_control.orientation.w = 1;
	marker_control.orientation.x = normal.x();
	marker_control.orientation.y = normal.y();
	marker_control.orientation.z = normal.z();
	//Ajout des controles associées
	marker_control.name = "move_plane";
	marker_control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
	_int_marker.controls.push_back(marker_control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	_server->insert(_int_marker);

	// 'commit' changes and send to all clients
	_server->applyChanges();
}

//Met à jour la zone d'erreur
void InteractiveObject::setErrorArea(double error)
{
	_showVisuals = true;

	_state.max_error = error;

	updateVisuals();
}

//Déplace le marker à une position donnée
void InteractiveObject::moveTo(const tf::Vector3& new_pos)
{
	tf::pointTFToMsg(new_pos, _int_marker.pose.position);
	_server->insert(_int_marker);
	_server->applyChanges();

	updateVisuals();
}

//Ajoute des infos visuelles (Zone d'erreur)
void InteractiveObject::addVisuals()
{
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
    _error_marker.header.frame_id = _int_marker.header.frame_id;
    _error_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    _error_marker.ns = _name;
    _error_marker.id = 0;
	_error_marker.type = visualization_msgs::Marker::SPHERE;
	_error_marker.scale.x = _int_marker.scale;
	_error_marker.scale.y = _int_marker.scale;
	_error_marker.scale.z = _int_marker.scale;
	_error_marker.color.r = 1;
	_error_marker.color.g = 0.5;
	_error_marker.color.b = 0.5;
	_error_marker.color.a = 0.5;
	_error_marker.pose = _int_marker.pose;

	//Publie le marker si possible
	if( ros::ok() && _visual_pub )
	{
		_visual_pub->publish(_error_marker);
	}
}

//Rafraichis les infos visuelles
void InteractiveObject::updateVisuals()
{
	// ROS_INFO_STREAM("Update visuals");
	if(!_showVisuals)
	{
		// ROS_INFO_STREAM("Suppresion des visuels");
		_error_marker.action = 2; //Suppresion du marker
	}
	else
	{
		_error_marker.action = 0; //Ajout/modification du marker

		_error_marker.scale.x = _state.max_error;
		_error_marker.scale.y = _state.max_error;
		_error_marker.scale.z = _state.max_error;

		_error_marker.pose = _int_marker.pose;
	}

	//Publie le marker si possible
	if( ros::ok() && _visual_pub )
	{
		_visual_pub->publish(_error_marker);
	}
}