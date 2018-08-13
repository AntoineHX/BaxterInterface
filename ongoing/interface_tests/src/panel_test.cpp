#include "panel_test.hpp"

//#include <geometry_msgs/Twist.h>

namespace interface_tests
{

// BEGIN_TUTORIAL
// Here is the implementation of the TestPanel class.  TestPanel
// has these responsibilities:
//
// - Act as a container for GUI elements DriveWidget and QLineEdit.
// - Publish command velocities 10 times per second (whether 0 or not).
// - Saving and restoring internal state from a config file.
//
// We start with the constructor, doing the standard Qt thing of
// passing the optional *parent* argument on to the superclass
// constructor, and also zero-ing the velocities we will be
// publishing.
TestPanel::TestPanel( QWidget* parent ): rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget( new QLabel( "Max Error:" ));
  max_error_editor = new QLineEdit;
  // max_error_editor->setValidator( new QDoubleValidator(0,MAX_ERROR,1000,max_error_editor)); // Ne gère pas les exceptions tout seul (retourne QValidator::Intermediate pour les valeurs hors bornes)
  topic_layout->addWidget( max_error_editor );
  setLayout( topic_layout );

  // Then create the control widget.
  // drive_widget_ = new DriveWidget;

  // // Lay out the topic field above the control widget.
  // QVBoxLayout* layout = new QVBoxLayout;
  // layout->addLayout( topic_layout );
  // layout->addWidget( drive_widget_ );
  // setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this TestPanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  // QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
 // connect( drive_widget_, SIGNAL( outputVelocity( float, float )), this, SLOT( setVel( float, float )));
  connect( max_error_editor, SIGNAL( editingFinished() ), this, SLOT( updateError() ));
  // connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Start the timer.
  // output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  // drive_widget_->setEnabled( false );

  _config_publisher = nh_.advertise<interface_tests::InterfaceConfigT>( "interface_config", 1 );
}

// setVel() is connected to the DriveWidget's output, which is sent
// whenever it changes due to a mouse event.  This just records the
// values it is given.  The data doesn't actually get sent until the
// next timer callback.
// void TestPanel::setVel( float lin, float ang )
// {
//   linear_velocity_ = lin;
//   angular_velocity_ = ang;
// }

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void TestPanel::updateError()
{
  //setTopic( max_error_editor->text() );
  setError(max_error_editor->text());
}

void TestPanel::setError( const QString& error )
{
  if( ros::ok() && _config_publisher )
  {
    interface_tests::InterfaceConfigT new_config;
    new_config.max_error = error.toDouble();
    _config_publisher.publish( new_config );
  }
}

// Set the topic name we are publishing to.
// void TestPanel::setTopic( const QString& new_topic )
// {
//   // Only take action if the name has changed.
//   if( new_topic != output_topic_ )
//   {
//     output_topic_ = new_topic;
//     // If the topic is the empty string, don't publish anything.
//     if( output_topic_ == "" )
//     {
//       _config_publisher.shutdown();
//     }
//     else
//     {
//       // The old ``_config_publisher`` is destroyed by this assignment,
//       // and thus the old topic advertisement is removed.  The call to
//       // nh_advertise() says we want to publish data on the new topic
//       // name.
//       _config_publisher = nh_.advertise<geometry_msgs::Twist>( output_topic_.toStdString(), 1 );
//     }
//     // rviz::Panel defines the configChanged() signal.  Emitting it
//     // tells RViz that something in this panel has changed that will
//     // affect a saved config file.  Ultimately this signal can cause
//     // QWidget::setWindowModified(true) to be called on the top-level
//     // rviz::VisualizationFrame, which causes a little asterisk ("*")
//     // to show in the window's title bar indicating unsaved changes.
//     Q_EMIT configChanged();
//   }

//   // Gray out the control widget when the output topic is empty.
//   // drive_widget_->setEnabled( output_topic_ != "" );
// }

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
// void TestPanel::sendVel()
// {
//   if( ros::ok() && _config_publisher )
//   {
//     geometry_msgs::Twist msg;
//     msg.linear.x = linear_velocity_;
//     msg.linear.y = 0;
//     msg.linear.z = 0;
//     msg.angular.x = 0;
//     msg.angular.y = 0;
//     msg.angular.z = angular_velocity_;
//     _config_publisher.publish( msg );
//   }
// }

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TestPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "Topic", output_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void TestPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString topic;
  if( config.mapGetString( "Topic", &topic ))
  {
    max_error_editor->setText( topic );
    updateError();
  }
}

} // end namespace

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(interface_tests::TestPanel,rviz::Panel )