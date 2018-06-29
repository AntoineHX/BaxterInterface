#include "InterfacePanel.hpp"

namespace rviz_interface
{

// BEGIN_TUTORIAL
// Here is the implementation of the InterfacePanel class.  InterfacePanel
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
InterfacePanel::InterfacePanel( QWidget* parent ): rviz::Panel( parent )
{
  // Next we lay out the "output topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* error_layout = new QHBoxLayout;
  error_layout->addWidget( new QLabel( "Max Error:" ));
  _max_error_editor = new QLineEdit;
  // max_error_editor->setValidator( new QDoubleValidator(0,MAX_ERROR,1000,max_error_editor)); // Ne gère pas les exceptions tout seul (retourne QValidator::Intermediate pour les valeurs hors bornes)
  error_layout->addWidget( _max_error_editor );
  //setLayout( error_layout );

  // Then create the control widget.
  // drive_widget_ = new DriveWidget;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  _objective_type_editor = new QCheckBox( "Precise Objective" );
  _show_visuals_editor = new QCheckBox( "Show Visuals" );
  _reset_button = new QPushButton("Reset", this);

  layout->addWidget( _reset_button );
  layout->addWidget( _objective_type_editor );
  layout->addLayout( error_layout );
  layout->addWidget( _show_visuals_editor );
  setLayout( layout );

  // Create a timer for sending the output.  Motor controllers want to
  // be reassured frequently that they are doing the right thing, so
  // we keep re-sending velocities even when they aren't changing.
  // 
  // Here we take advantage of QObject's memory management behavior:
  // since "this" is passed to the new QTimer as its parent, the
  // QTimer is deleted by the QObject destructor when this InterfacePanel
  // object is destroyed.  Therefore we don't need to keep a pointer
  // to the timer.
  // QTimer* output_timer = new QTimer( this );

  // Next we make signal/slot connections.
  connect( _max_error_editor, SIGNAL( editingFinished() ), this, SLOT( updateError() ));
  connect( _objective_type_editor, SIGNAL( stateChanged(int) ), this, SLOT( updateType(int) ));
  connect(_reset_button, SIGNAL (released()), this, SLOT (handleResetButton()));
  connect( _show_visuals_editor, SIGNAL( stateChanged(int) ), this, SLOT( updateVisuals(int) ));
  // connect( output_timer, SIGNAL( timeout() ), this, SLOT( sendVel() ));

  // Start the timer.
  // output_timer->start( 100 );

  // Make the control widget start disabled, since we don't start with an output topic.
  // drive_widget_->setEnabled( false );
  _max_error_editor->setEnabled( false );

  _config_publisher = _nh.advertise<rviz_interface::InterfaceConfig>( "/RvizInterface/interface_config", 1 );
}

InterfacePanel::~InterfacePanel()
{
  delete _max_error_editor;
  delete _objective_type_editor;
}

// Read the topic name from the QLineEdit and call setTopic() with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void InterfacePanel::updateError()
{
  current_config.max_error = _max_error_editor->text().toDouble();

  if( ros::ok() && _config_publisher )
  {
    _config_publisher.publish( current_config );
  }
}

void InterfacePanel::updateType(int state)
{
  current_config.objective_type = state;

  _max_error_editor->setEnabled( state ); //Active l'editeur d'erreur si state>0 (ie Objectif précis)
  if(! state) //Pas d'objectif précis
  {
    _show_visuals_editor->setCheckState( Qt::Unchecked ); //Affichage de la zone d'erreur inutile
    current_config.show_visuals = 0;
  }

  //Publication si possible
  if( ros::ok() && _config_publisher )
  {
    _config_publisher.publish( current_config );
  }
}

void InterfacePanel::updateVisuals(int state)
{
  current_config.show_visuals = state;
  if( ros::ok() && _config_publisher )
  {
    _config_publisher.publish( current_config );
  }
}

void InterfacePanel::handleResetButton()
{
  current_config.follow_object = true;
  if( ros::ok() && _config_publisher )
  {
    _config_publisher.publish( current_config );
  }
  current_config.follow_object = false;
}

// void InterfacePanel::setError( const QString& error )
// {
//   current_config.max_error = error.toDouble();
//   if( ros::ok() && _config_publisher )
//   {
//     _config_publisher.publish( current_config );
//   }
// }

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
// void InterfacePanel::save( rviz::Config config ) const
// {
//   rviz::Panel::save( config );
//   config.mapSetValue( "Error", _max_error );
// }

// // Load all configuration data for this panel from the given Config object.
// void InterfacePanel::load( const rviz::Config& config )
// {
//   rviz::Panel::load( config );
//   QString error;
//   if( config.mapGetString( "Error", &error ))
//   {
//     _max_error_editor->setText( error );
//     updateError();
//   }
// }

} // end namespace

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_interface::InterfacePanel,rviz::Panel )