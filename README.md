# BaxterInterface
Bricks for an Rviz based interface for objective sending.
Maintainer: Antoine Harlé antoine.harle@etu.upmc.fr

## What is this ?

This is a ROS package implementing two working ROS modules : rviz_interface & asift_matching
It also include other ongoing non-ROS modules, mostly for test purposes.

### rviz_interface

This ROS modules implements markers and a panel for Rviz for sending StateSpace messages.

### asift_matching


* texte reference generation :

## Status 
### Done

### Todo
REGLER LE PROBLEME DE FRAME ...
Problème dans le filtrage : nombre de points _> probleme de reset du nombre

## How to use it ?

### Requirements

* ROS Indigo (probably works in other versions too)
 * PCL
 * QT 5
* Eigen 3

### Usage

* Clone the repository to your catkin src
* catkin_make

#### Simple Rviz interface
* Run : roslaunch rviz_interface interface.launch
* Setup rviz :
 - set fixed frame to camera_rgb_optical_frame.
 - add an InteractiveMarker and set topic to /RvizInterface/update.
 - add a Marker and set topic to vizualization_topic parameter.
 - add the Rviz interface panel.

#### Rviz interface with simple moving tracker
* Run :
 - roslaunch openni_launch openni.launch 
 - roslaunch rviz_interface interface_plannar_seg.launch
* Setup rviz :
 - set fixed frame to camera_rgb_optical_frame.
 - add an InteractiveMarker and set topic to /RvizInterface/update.
 - add a Marker and set topic to vizualization_topic parameter.
 - add the Rviz interface panel.
 - add a PointCloud2 and set topic to /camera/depth_registered/points.

#### ASIFT matching (no vizualisation)

* Run :
 - roslaunch openni_launch openni.launch 
 - asift_matching : roslaunch asift_matching asift_match.launch

#### ASIFT matching & Rviz interface
* Run :
 - roslaunch openni_launch openni.launch 
 - roslaunch rviz_interface interface_matching.launch
* Setup rviz :
 - set fixed frame to camera_rgb_optical_frame.
 - add an InteractiveMarker and set topic to /RvizInterface/update.
 - add a Marker and set topic to vizualization_topic parameter.
 - add the Rviz interface panel.
 - add a PointCloud2 and set topic to /camera/depth_registered/points.

### Parameters

* The topic names are defined as parameters in the launchfiles.

* rviz_interface parameters are defined as parameters in the rviz_interface launchfiles : Interface objects are defined by their names and types (0-Default, 1-Button, 2-2D object, 3-3D object) (see StateSpace.msg for more details).

* asift_matching parameters are defined as parameters in the asift_matching launchfiles : 
-> References are defined by the path to the references files can be a txt file (faster - see texte reference generation in asift_matching section) or a series of images files (slower - currently not working).
-> Tracked object is defined by the object name (must be one of the interface object to work properly with rviz_interface).
-> Number of tilts is defined by an integer value (1-no tilt, 7-recommended value). Higher value means slower process.
-> Standaed deviation filter coefficient is defined by a real value. Keep (1-68%/2-95%/3-99%) of the points statistically. 0: no filtering (default).
