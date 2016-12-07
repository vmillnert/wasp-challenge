#include <stdio.h>
#include <stdlib.h>

#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QWidget>
#include <QLabel>

#include "std_msgs/String.h"

#include "wasp_panel.h"
namespace operator_panel
{

WaspPanel::WaspPanel(QWidget* parent )
  : rviz::Panel( parent )
{

  // Add buttons
  plan_button_ = new QPushButton("Start");
  cancel_button_ = new QPushButton("Stop");
  pause_button_ = new QPushButton("Pause");

  plan_button_->setToolTip("Test");
  cancel_button_->setToolTip("Cancel the plan. Finishes the current actions.");
  pause_button_->setToolTip("Finishes current actions, then pauses the exection of the plan. Click Pause again to continue.");

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget( plan_button_ );
  layout->addWidget( cancel_button_ );
  layout->addWidget( pause_button_ );

  setLayout( layout );

  connect( plan_button_, SIGNAL( clicked()), this, SLOT( sendPlan() ));
  connect( cancel_button_, SIGNAL( clicked() ), this, SLOT( sendCancel() ));
  connect( pause_button_, SIGNAL( clicked() ), this, SLOT( sendPause() ));

  planner_publisher_ = nh_.advertise<std_msgs::String>( "/kcl_rosplan/planning_commands", 1 );

}

void WaspPanel::sendPause() {
  std_msgs::String msg;
  msg.data = "pause";
  planner_publisher_.publish(msg);
  ROS_INFO("\033[31m %s\033[0m", msg.data.c_str());
}

void WaspPanel::sendPlan()
{
  std_msgs::String msg;
  msg.data = "plan";
  planner_publisher_.publish(msg);
  ROS_INFO("\033[31m %s\033[0m", msg.data.c_str());
}

void WaspPanel::sendCancel()
{
  std_msgs::String msg;
  msg.data = "cancel";
  planner_publisher_.publish(msg);
  ROS_INFO("\033[31m %s\033[0m", msg.data.c_str());
}

} // end namespace operator_panel

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(operator_panel::WaspPanel,rviz::Panel )
// END_TUTORIAL
