/*
 * Rviz panel for the configuration of the RvizInterface.
 * @author : antoine.harle@etu.upmc.Fr
 * @see : RvizInterface.cpp/.hpp
 */

#include "InterfacePanel.hpp"

namespace rviz_interface
{

/* Constructor
 * parent : QT widget parent.
 */
InterfacePanel::InterfacePanel( QWidget* parent ): rviz::Panel( parent )
{
  QHBoxLayout* error_layout = new QHBoxLayout;
  error_layout->addWidget( new QLabel( "Max Error:" ));
  _max_error_editor = new QLineEdit;
   error_layout->addWidget( _max_error_editor );

  QVBoxLayout* layout = new QVBoxLayout;
  _objective_type_editor = new QCheckBox( "Precise Objective" );
  _show_visuals_editor = new QCheckBox( "Show Visuals" );
  _reset_button = new QPushButton("Reset", this);

  layout->addWidget( _reset_button );
  layout->addWidget( _objective_type_editor );
  layout->addLayout( error_layout );
  layout->addWidget( _show_visuals_editor );
  setLayout( layout );


  // Next we make signal/slot connections.
  connect( _max_error_editor, SIGNAL( editingFinished() ), this, SLOT( updateError() ));
  connect( _objective_type_editor, SIGNAL( stateChanged(int) ), this, SLOT( updateType(int) ));
  connect(_reset_button, SIGNAL (released()), this, SLOT (handleResetButton()));
  connect( _show_visuals_editor, SIGNAL( stateChanged(int) ), this, SLOT( updateVisuals(int) ));

  //Error editor disabled by default
  _max_error_editor->setEnabled( false );

  _config_publisher = _nh.advertise<rviz_interface::InterfaceConfig>( "/RvizInterface/interface_config", 1 );
}

/* Destructor
 */
InterfacePanel::~InterfacePanel()
{
  delete _max_error_editor;
  delete _objective_type_editor;
}

/* Update the error in the current_configuration & publish the configuration.
 */
void InterfacePanel::updateError()
{
  current_config.max_error = _max_error_editor->text().toDouble();

  if( ros::ok() && _config_publisher )
  {
    _config_publisher.publish( current_config );
  }
}

/* Update the objective type (ie Precise or not) in the current_configuration & publish the configuration.
 * state : New objective type (1 : Precise).
 */
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

/* Update the visual flag (ie Show visuals or not) in the current_configuration & publish the configuration.
 * state : Visual flag (1 : Show visuals).
 */
void InterfacePanel::updateVisuals(int state)
{
  current_config.show_visuals = state;
  if( ros::ok() && _config_publisher )
  {
    _config_publisher.publish( current_config );
  }
}

/* Send a signal through the configuration telling the subscriber to follow the object it's linked to.
*/
void InterfacePanel::handleResetButton()
{
  current_config.follow_object = true;
  if( ros::ok() && _config_publisher )
  {
    _config_publisher.publish( current_config );
  }
  current_config.follow_object = false;
}

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