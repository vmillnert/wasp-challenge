#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"

#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "std_msgs/String.h"

#include "wasp_panel.h"
namespace operator_panel
{

WaspPanel::WaspPanel(QWidget* parent )
  : rviz::Panel( parent )
{

  // Add start button
  start_button_ = new QPushButton("Start");
  stop_button_ = new QPushButton("Stop");

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget( start_button_ );
  layout->addWidget( stop_button_ );

  setLayout( layout );

  connect( start_button_, SIGNAL( clicked()), this, SLOT( sendStart() ));
  connect( stop_button_, SIGNAL( clicked() ), this, SLOT( sendStop() ));


  // planner_publisher_ = nh_.advertise<std_msgs::String>( "/rosout", 1 );

}

void WaspPanel::sendStart()
{
  std_msgs::String msg;
  msg.data = "\033[31m sendStart()\033[0m";
  chatter_pub_.publish(msg);
  ROS_INFO("%s", msg.data.c_str());
}

void WaspPanel::sendStop()
{
  std_msgs::String msg;
  msg.data = "\033[31m sendStop()\033[0m";
  chatter_pub_.publish(msg);
  ROS_INFO("%s", msg.data.c_str());
}

} // end namespace operator_panel

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(operator_panel::WaspPanel,rviz::Panel )
// END_TUTORIAL
