#ifndef INTERACTIVEOBJECT_HPP
#define INTERACTIVEOBJECT_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <tf/tf.h>

#include <rviz_interface/StateSpace.h>

using namespace visualization_msgs;

class InteractiveObject
{
protected:
	unsigned int _objectID; //Identifiant de l'objet
	static unsigned int nextObjectID;
	std::string _name;
	unsigned int _type;
	//std::vector<visualization_msgs::InteractiveMarker> _int_markers;
	InteractiveMarker _int_marker;

	interactive_markers::InteractiveMarkerServer* _server;
	rviz_interface::StateSpace _state;

	ros::Publisher* _objective_pub;

public:
	InteractiveObject(ros::Publisher* objective_pub, interactive_markers::InteractiveMarkerServer* server, const std::string& name, unsigned int type, unsigned int shape, const tf::Vector3& position);

	void createInteractiveMarker(Marker& marker, const tf::Vector3& position);

	void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback );

	void addButtoncontrol();
	void add6DOFcontrol();
	void add3DOFcontrol();

	//std::vector<visualization_msgs::InteractiveMarker>& markers(){ return _int_markers;}
	InteractiveMarker& marker(){ return _int_marker;}
	rviz_interface::StateSpace& state(){ return _state;}
};

#endif