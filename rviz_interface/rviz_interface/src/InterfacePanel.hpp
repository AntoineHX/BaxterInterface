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
// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
class InterfacePanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  InterfacePanel( QWidget* parent = 0 );
  ~InterfacePanel();

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  // virtual void load( const rviz::Config& config );
  // virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:

	// void setError( const QString& error );

  // Here we declare some internal slots.
protected Q_SLOTS:


  void updateError();
  void updateType(int state);
  void handleResetButton();


  // Then we finish up with protected member variables.
protected:

  rviz_interface::InterfaceConfig current_config;

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* _max_error_editor;
  QCheckBox* _objective_type_editor;
  QPushButton* _reset_button;

  // The current name of the output topic.
  // QString _max_error;

  // The ROS publisher for the command velocity.
  ros::Publisher _config_publisher;

  // The ROS node handle.
  ros::NodeHandle _nh;

  // The latest velocity values from the drive widget.
  // float linear_velocity_;
  // float angular_velocity_;
  // END_TUTORIAL
};

} // end namespace rviz_plugin_tutorials

#endif