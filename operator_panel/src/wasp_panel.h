#ifndef WASP_PANEL_H
#define WASP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

class QPushButton;

namespace operator_panel
{


class WaspPanel: public rviz::Panel
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
  WaspPanel(QWidget* parent = 0 );

  // Here we declare some internal slots.
protected Q_SLOTS:

  void sendPlan();
  void sendCancel();
  void sendPause();
  void readWorldState();
  void worldStatePlan();
  // Then we finish up with protected member variables.
protected:
  // Start button
  QPushButton* plan_button_;

  // Stop button
  QPushButton* cancel_button_;

  // Pause button
  QPushButton* pause_button_;

  // Read World State button
  QPushButton* read_world_state_button_;

  // World state plan button
  QPushButton* world_state_plan_button_;

  // Publisher for the planning commands
  ros::Publisher planner_publisher_;

  // ROS Service client for /world_state
  ros::ServiceClient world_state_read_service_;
  ros::ServiceClient world_state_plan_service_;

  // The ROS node handle.
  ros::NodeHandle nh_;

};

} // end namespace operator_panel

#endif // WASP_PANEL_H
