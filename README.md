# wasp-challenge
WASP Challenge for the Autonomous Systems Course Fall 2016

For the ROSPlan Package depends on the following package to fully function.
sudo apt-get install flex ros-indigo-mongodb-store ros-indigo-tf2-bullet freeglut3-dev

---Turtlebot packages---
sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs

Directory Contents
-----------------------------------------------
src/
  bebop_controller/	Provides an action server and controllers for the Bebop drone.
  coordinator/		Provides a coordinator node which distributes actions (from ROSPlan etc) to agents. The coordinator should also feedback information to the planner state.
  keys/			SSH keys used in the lab setup. THESE KEYS MUST NOT BE USED WITH ANY SENSITIVE SYSTEM!
  rmote/		Remote roslaunch
  util/			Lab environment utilities.
  wasp_g1_start/	The turtlebot files.
  world/	 	Contains code to locate things in the world. This currently entails the April tag detection used to position agents.

  actionlib/	 	git submodule providing a standardized interface for interfacing with preepmtable tasks. http://http://wiki.ros.org/actionlib
  bebop_autonomy/  	git submodule ROS driver for the Parrot Bebop drone. http://http://wiki.ros.org/bebop_autonomy
  multiple_bebops/	git submodule used to connect drones to a network access point. https://github.com/tnaegeli/multiple_bebops
  occupancy_grid_utils/	git submodule ROSPlan dependency. https://github.com/KCL-Planning/rosplan
  rosplan/		git submodule the planning framework ROSPlan. https://github.com/KCL-Planning/rosplan
  rplidar_ros		git submodule LiDAR driver package. https://github.com/robopeak/rplidar_ros.git
