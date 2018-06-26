/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
//#include <tf/tf.h>

#include <interface_tests/StateSpaceT.h>

using namespace visualization_msgs;

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	//// SEND OBJECTIVE ////
	if(feedback->event_type == InteractiveMarkerFeedback::BUTTON_CLICK)
	{
		//TRES DEGEU -> FAIRE EN OBJET
		//ros::Publisher objective_pub = n.advertise<interface_tests::StateSpaceT>("state_objective", 10);

		interface_tests::StateSpaceT obj;
		obj.name ="Objective_test";
		// obj.state_type = interface_tests::STATE_2D;
		obj.state_type = 1;
		//obj.dimension = 3;
		obj.discrete_data.push_back(1);
		obj.discrete_data.push_back(false);
		obj.real_data.push_back(10.2);
		obj.description="Test du message StateSpaceT !";

		//objective_pub.publish(obj);
	}

	//// PRINT INFO ////
  // ROS_INFO_STREAM( feedback->marker_name << " is now at "
  //     << feedback->pose.position.x << ", " << feedback->pose.position.y
  //     << ", " << feedback->pose.position.z );

	std::ostringstream s;
  	s << "Feedback from marker '" << feedback->marker_name << "' "<< " / control '" << feedback->control_name << "'";

  	std::ostringstream mouse_point_ss;
	if( feedback->mouse_point_valid )
	{
	  	mouse_point_ss << " at " << feedback->mouse_point.x
	                   << ", " << feedback->mouse_point.y
	                   << ", " << feedback->mouse_point.z
	                   << " in frame " << feedback->header.frame_id;
	}
	
	switch ( feedback->event_type )
	{
	   	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
	    ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
	    break;

	    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_test");

  ros::NodeHandle n;

  //Create Objective topic
  ros::Publisher objective_pub = n.advertise<interface_tests::StateSpaceT>("state_objective", 10);

  //Test du topic
 	interface_tests::StateSpaceT obj;
	obj.name ="Objective_test";
	// obj.state_type = interface_tests::STATE_2D;
	obj.state_type = 1;
	//obj.dimension = 3;
	obj.discrete_data.push_back(1);
	obj.discrete_data.push_back(false);
	obj.real_data.push_back(10.2);
	obj.description="Test du message StateSpaceT !";

	objective_pub.publish(obj);

  ROS_INFO_STREAM("Starting Marker Test");
  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("marker_test");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "my_marker";
  int_marker.description = "Test Control";

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

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true; //Toujours visible hors du mode interaction
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl new_control;

  new_control.name = "move_test";

  //MODE 3D fonctionnel ???
  // new_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;  

  // add the control to the interactive marker
  //int_marker.controls.push_back(new_control);

    //Orientation du vecteur
  // new_control.orientation.w = 1;
  // new_control.orientation.x = 0;
  // new_control.orientation.y = 1;
  // new_control.orientation.z = 0;

    // new_control.name = "rotate_y";
    // new_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    // int_marker.controls.push_back(new_control);

    // new_control.name = "move_plane_y";
    // new_control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE; //Attention par défaut utilise un halo (comme le rotate Axis) peut etre ignorer -> utiliser MOVE_ROTATE dans ce cas
    // int_marker.controls.push_back(new_control); 

    // new_control.name = "move_rotate_y";
    // new_control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE; //Difficile pour des mouvements précis
    // int_marker.controls.push_back(new_control); 

    //Boutton
  // new_control.interaction_mode = InteractiveMarkerControl::BUTTON;
  // new_control.name = "button_control";

  // Marker marker = makeBox( int_marker );
  // new_control.markers.push_back( marker );
  // new_control.always_visible = true;
  // int_marker.controls.push_back(new_control);

  //TEST 6DOF

  //int_marker.controls[0].interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D; //Deplacement par "grab" de la souris (CTRL + Souris = rotation)
  int_marker.controls[0].interaction_mode = InteractiveMarkerControl::BUTTON; //Ajout fonctionalité Boutton (Validation ?)

  new_control.orientation_mode = InteractiveMarkerControl::FIXED;
  //Orientation du vecteur
  new_control.orientation.w = 1;
  new_control.orientation.x = 1;
  new_control.orientation.y = 0;
  new_control.orientation.z = 0;
  //Ajout des controles associées
  new_control.name = "rotate_x";
  new_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(new_control);
  new_control.name = "move_x";
  new_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(new_control);

  //Orientation du vecteur
  new_control.orientation.w = 1;
  new_control.orientation.x = 0;
  new_control.orientation.y = 1;
  new_control.orientation.z = 0;
  //Ajout des controles associées
  new_control.name = "rotate_y";
  new_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(new_control);
  new_control.name = "move_y";
  new_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(new_control);

  //Orientation du vecteur
  new_control.orientation.w = 1;
  new_control.orientation.x = 0;
  new_control.orientation.y = 0;
  new_control.orientation.z = 1;
  //Ajout des controles associées
  new_control.name = "rotate_z";
  new_control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(new_control);
  new_control.name = "move_z";
  new_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(new_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();

  // ros::Rate loop_rate(10);
  // while (ros::ok())
  // {
  // 	objective_pub.publish(obj);
  // 	 ros::spinOnce();
  // 	 loop_rate.sleep();
  // }
}
// %Tag(fullSource)%
