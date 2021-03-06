#ifndef PANEL_TEST_HPP
#define PANEL_TEST_HPP

//#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
//#endif

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
// #include <QDoubleValidator>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

#include <interface_tests/InterfaceConfigT.h>

#define MAX_ERROR 999999999

class QLineEdit;

namespace interface_tests
{
// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
class TestPanel: public rviz::Panel
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
  TestPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:
  // The control area, DriveWidget, sends its output to a Qt signal
  // for ease of re-use, so here we declare a Qt slot to receive it.
  // void setVel( float linear_velocity_, float angular_velocity_ );

  // In this example setTopic() does not get connected to any signal
  // (it is called directly), but it is easy to define it as a public
  // slot instead of a private function in case it would be useful to
  // some other user.
  // void setTopic( const QString& topic );
	void setError( const QString& error );

  // Here we declare some internal slots.
protected Q_SLOTS:
  // sendvel() publishes the current velocity values to a ROS
  // topic.  Internally this is connected to a timer which calls it 10
  // times per second.
  // void sendVel();

  // updateError() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  void updateError();

  // Then we finish up with protected member variables.
protected:

  // One-line text editor for entering the outgoing ROS topic name.
  QLineEdit* max_error_editor;

  // The current name of the output topic.
  QString output_topic_;

  // The ROS publisher for the command velocity.
  ros::Publisher _config_publisher;

  // The ROS node handle.
  ros::NodeHandle nh_;

  // The latest velocity values from the drive widget.
  // float linear_velocity_;
  // float angular_velocity_;
  // END_TUTORIAL
};

} // end namespace rviz_plugin_tutorials

#endif