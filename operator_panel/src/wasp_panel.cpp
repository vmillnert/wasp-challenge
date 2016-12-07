#include <stdio.h>
#include <stdlib.h>

#include <QPainter>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include "std_srvs/Empty.h"
#include "std_msgs/String.h"

#include "wasp_panel.h"

namespace operator_panel
{

WaspPanel::WaspPanel(QWidget* parent )
  : rviz::Panel( parent )
{

  // Add buttons
  plan_button_ = new QPushButton("KCL Plan");
  cancel_button_ = new QPushButton("KCL Cancel");
  pause_button_ = new QPushButton("KCL Pause");
  read_world_state_button_ = new QPushButton("Read World State");
  world_state_plan_button_ = new QPushButton("Plan World State");

  plan_button_->setToolTip("Plan/Replan.");
  cancel_button_->setToolTip("Cancel the plan. Finishes the current actions.");
  pause_button_->setToolTip("Finishes current actions, then pauses the exection of the plan. Click Pause again to continue.");
  read_world_state_button_->setToolTip("Reads the world state again.");
  world_state_plan_button_->setToolTip("Fill the knowledge base. And plan.");

  QHBoxLayout* h_layout = new QHBoxLayout;

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout_left = new QVBoxLayout;
  layout_left->addWidget( plan_button_ );
  layout_left->addWidget( cancel_button_ );
  layout_left->addWidget( pause_button_ );

  QVBoxLayout* layout_right = new QVBoxLayout;
  layout_right->addWidget( read_world_state_button_ );
  layout_right->addWidget( world_state_plan_button_ );

  h_layout->addLayout(layout_left);
  h_layout->addLayout(layout_right);

  setLayout( h_layout );

  connect( plan_button_, SIGNAL( clicked()), this, SLOT( sendPlan() ));
  connect( cancel_button_, SIGNAL( clicked() ), this, SLOT( sendCancel() ));
  connect( pause_button_, SIGNAL( clicked() ), this, SLOT( sendPause() ));
  connect( read_world_state_button_, SIGNAL( clicked() ), this, SLOT( readWorldState() ));
  connect( world_state_plan_button_, SIGNAL( clicked() ), this, SLOT( worldStatePlan() ));

  planner_publisher_ = nh_.advertise<std_msgs::String>( "/kcl_rosplan/planning_commands", 1 );
  world_state_read_service_ = nh_.serviceClient<std_srvs::Empty>("/world_state/read_world_config");
  world_state_plan_service_ = nh_.serviceClient<std_srvs::Empty>("/world_state/plan");

}

void WaspPanel::readWorldState() {
  std_srvs::Empty srv;
  world_state_read_service_.call(srv);
  ROS_INFO("\033[32mrosservice call /world_state/read_world_config\033[0m");
}

void WaspPanel::worldStatePlan() {
  std_srvs::Empty srv;
  world_state_plan_service_.call(srv);
  ROS_INFO("\033[32mrosservice call /world_state/plan\033[0m");
}

void WaspPanel::sendPause() {
  std_msgs::String msg;
  msg.data = "pause";
  planner_publisher_.publish(msg);
  ROS_INFO("\033[32m%s\033[0m", msg.data.c_str());
}

void WaspPanel::sendPlan()
{
  std_msgs::String msg;
  msg.data = "plan";
  planner_publisher_.publish(msg);
  ROS_INFO("\033[32m%s\033[0m", msg.data.c_str());
}

void WaspPanel::sendCancel()
{
  std_msgs::String msg;
  msg.data = "cancel";
  planner_publisher_.publish(msg);
  ROS_INFO("\033[32m%s\033[0m", msg.data.c_str());
}

} // end namespace operator_panel

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(operator_panel::WaspPanel,rviz::Panel )
// END_TUTORIAL
