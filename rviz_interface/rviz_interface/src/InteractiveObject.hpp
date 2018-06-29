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
	bool _showVisuals;
	bool _followObject;
	//std::vector<visualization_msgs::InteractiveMarker> _int_markers;
	InteractiveMarker _int_marker;
	// std::vector<Marker> _visual_markers;
	InteractiveMarkerControl visual_container;

	interactive_markers::InteractiveMarkerServer* _server;
	rviz_interface::StateSpace _state;

	ros::Publisher* _objective_pub;

public:
	InteractiveObject(interactive_markers::InteractiveMarkerServer* server, const std::string& name, unsigned int type, unsigned int shape, const tf::Vector3& position);

	void createInteractiveMarker(Marker& marker, const tf::Vector3& position);

	void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback );

	void addButtoncontrol();
	void add6DOFcontrol();
	void add3DOFcontrol();

	void addVisuals();

	void setObjectivePublisher(ros::Publisher* objective_pub){ _objective_pub=objective_pub;}
	// void setVisualizationPublisher(ros::Publisher* visualization_pub){ _visualization_pub=visualization_pub;}
	void setErrorArea(double error);

	void moveTo(const tf::Vector3& new_pos);

	//std::vector<visualization_msgs::InteractiveMarker>& markers(){ return _int_markers;}
	InteractiveMarker& marker(){ return _int_marker;}
	rviz_interface::StateSpace& state(){ return _state;}
	bool isFollowing() const{ return _followObject;}
	void follow(){ _followObject = true;}
};

#endif