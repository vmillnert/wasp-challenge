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

  void sendStart();
  void sendStop();
  // Then we finish up with protected member variables.
protected:
  // Start button
  QPushButton* start_button_;

  // Stop button
  QPushButton* stop_button_;

  // Publisher for the planning commands
  //ros::Publisher planner_publisher_;
  ros::Publisher chatter_pub_;

  // The ROS node handle.
  ros::NodeHandle nh_;

};

} // end namespace operator_panel

#endif // WASP_PANEL_H
