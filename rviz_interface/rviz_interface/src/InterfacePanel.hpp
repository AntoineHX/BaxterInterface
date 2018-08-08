/*
 * Rviz panel for the configuration of the RvizInterface.
 * @author : antoine.harle@etu.upmc.Fr
 * @see : RvizInterface.cpp/.hpp
 */

#ifndef INTERFACE_PANEL_HPP
#define INTERFACE_PANEL_HPP

//#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
//#endif

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
// #include <QDoubleValidator>
#include <QCheckBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <rviz_interface/InterfaceConfig.h>

#define MAX_ERROR 999999999

class QLineEdit;

namespace rviz_interface
{

class InterfacePanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  InterfacePanel( QWidget* parent = 0 );
  ~InterfacePanel();

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  // virtual void load( const rviz::Config& config );
  // virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:

  // Here we declare some internal slots.
protected Q_SLOTS:


  void updateError(); //Update the error in the current_configuration & publish the configuration.
  void updateType(int state); //Update the objective type (ie Precise or not) in the current_configuration & publish the configuration.
  void updateVisuals(int state); //Update the visual flag (ie Show visuals or not) in the current_configuration & publish the configuration.
  void handleResetButton(); //Send a signal through the configuration telling the subscriber to follow the object it's linked to.

protected:

  rviz_interface::InterfaceConfig current_config; //Current configuration which is sent at every interaction with the panel.

  QLineEdit* _max_error_editor; //Line editor used to let user cusomize the maximum error allowed.
  QCheckBox* _objective_type_editor; //CheckBox used to modify the objective type.
  QCheckBox* _show_visuals_editor; //CheckBox used to choose if the error area should be shown.
  QPushButton* _reset_button; //Button to reattach a marker to it's linked object.

  //Publisher
  ros::Publisher _config_publisher;

  // The ROS node handle.
  ros::NodeHandle _nh;

};

} // end namespace rviz_plugin_tutorials

#endif