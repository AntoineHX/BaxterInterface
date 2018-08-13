/*
 * Rviz interactive object.
 * @author : antoine.harle@etu.upmc.Fr
 * @see : RvizInterface.cpp/.hpp
 */

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
	std::string _name; //Nom de l'objet
	unsigned int _type; //Type de l'objet
	bool _showVisuals; //Indique si les infos visuelles sont affichées
	bool _followObject; //Indique si le marker suit l'objet

	InteractiveMarker _int_marker; //Marker interactif
	Marker _error_marker; //Marker visuel de zone d'erreur

	interactive_markers::InteractiveMarkerServer* _server; //Serveur d'InteractiveMaker

	rviz_interface::StateSpace _state; //Etat "mémoire"

	//Publisher ROS
	ros::Publisher* _objective_pub;
	ros::Publisher* _visual_pub;

public:
	InteractiveObject(interactive_markers::InteractiveMarkerServer* server, const std::string& name, unsigned int type, unsigned int shape, const tf::Vector3& position = tf::Vector3(0,0,0));

	//Construit un InteractiveMarker
	void createInteractiveMarker(Marker& marker, const tf::Vector3& position = tf::Vector3(0,0,0));

	//Fonction callback du serveur d' InteractiveMarker
	void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback );
	
	//Ajoute une fonction bouton
	void addButtoncontrol();
	//Ajoute des controle pour 6DOF (Déplacement dans l'espace)
	void add6DOFcontrol();
	//Ajoute des controle pour 3DOF (Déplacement dans le plan x/z)
	void add3DOFcontrol();
	//Ajoute des controle pour 3DOF (Déplacement dans le plan)
	void add3DOFcontrol(const tf::Vector3& normal);

	//Ajoute des infos visuelles (Zone d'erreur)
	void addVisuals();
	//Rafraichis les infos visuelles
	void updateVisuals();

	//Met à jour la zone d'erreur
	void setErrorArea(double error);

	//Déplace le marker à une position donnée
	void moveTo(const tf::Vector3& new_pos);

	//Accesseurs
	const std::string& name() const{ return _name;}
	void setObjectivePublisher(ros::Publisher* objective_pub){ _objective_pub=objective_pub;}
	void setVisualizationPublisher(ros::Publisher* visualization_pub){ _visual_pub=visualization_pub;}
	InteractiveMarker& int_marker(){ return _int_marker;}
	Marker& err_marker(){ return _error_marker;}
	rviz_interface::StateSpace& state(){ return _state;}
	bool isFollowing() const{ return _followObject;}
	void follow(bool follow = true){ _followObject = follow;}
	void showVisuals(bool showVisuals = true){ _showVisuals=showVisuals; updateVisuals();}
};

#endif