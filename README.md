# wasp-challenge
WASP Challenge for the Autonomous Systems Course Fall 2016.

## Group 1 - Lund University
* David Gillsjö
* Victor Millnert
* Christian Nelson
* Hamed Sadeghi
* Per Skarin
* Alfred Åkesson



## Dependencies

Using [Ubuntu 14.04](http://releases.ubuntu.com/14.04/) and [ROS Indigo Full](http://wiki.ros.org/indigo/Installation/Ubuntu).

### Turtlebot packages
```
sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
```

### ROSPlan
For the ROSPlan Package depends on the following package to fully function.

```
sudo apt-get install flex ros-indigo-mongodb-store ros-indigo-tf2-bullet freeglut3-dev
```

It is important to don't have a mongodb service running. Ubuntu has this as default. If

```
roslaunch wasp_g1_start rosplan_test.launch
```

fails, then it is most likely due too a running instance of mongodb. Then run

```
sudo service mongodb stop
```

### Simulator
The simulator uses the Parrot AR.Drone, not Bebop, and the thinc_simulator package. There is a bash script `startsim.sh` on the source root which launches a simulator environment.

```
sudo apt-get install ros-indigo-turtlebot-gazebo ros-indigo-ardrone-autonomy ros-indigo-parrot-arsdk ros-indigo-zeroconf-avahi
```


## Directory Contents
src/
  * `startsim.sh`       Starts simulator environment. It's a bash script since timing is an issue.
  * `bebop_controller/`	Provides an action server and controllers for the Bebop drone.
  * `coordinator/`		  Provides a coordinator node which distributes actions (from ROSPlan etc) to agents. The coordinator should also feedback information to the planner state.
  * `keys/`			        SSH keys used in the lab setup. THESE KEYS MUST NOT BE USED WITH ANY SENSITIVE SYSTEM!
  * `rmote/`		        Remote roslaunch
  * `util/`			        Lab environment utilities.
  * `wasp_g1_start/`	  The turtlebot files.
  * `world/`	 	        Contains code to locate things in the world. This currently entails the April tag detection used to position agents.

---

  * `actionlib/`
    * git submodule providing a standardized interface for interfacing with preepmtable tasks.
    * http://wiki.ros.org/actionlib
  * `bebop_autonomy/`
    * git submodule (__forked__) ROS driver for the Parrot Bebop drone.
    * http://wiki.ros.org/bebop_autonomy
  * `multiple_bebops/`
    * git submodule used to connect drones to a network access point.
    * https://github.com/tnaegeli/multiple_bebops
  * `occupancy_grid_utils/`
    * git submodule ROSPlan dependency.
    * https://github.com/KCL-Planning/rosplan
  * `rosplan/`
    * git submodule (__forked__); the planning framework ROSPlan.
    * https://github.com/KCL-Planning/rosplan
  * `rplidar_ros/`
    * git submodule; LiDAR driver package.
    * https://github.com/robopeak/rplidar_ros.git
  * `thinc_simulator/`
    * git submodule for AR-Drone simulation.
    * https://github.com/thinclab/thinc_simulator
