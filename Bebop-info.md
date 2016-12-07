## Connect the Bebop to our wifi

To make the Bebop (red) connect to our wifi `WASP1-24` using
ip-address `10.42.0.102` press the on/off-button for 4 seconds after
it has booted.

You should be able to ping the bebop using `$ping 10.42.0.102` or `$ping bebop`

*Note* that you will have to change a launch-file in `bebop_autonomy` to run ros using this new ip-address:

in `/bebop_autonomy/bebop_driver/launch/bebop_nodelet.launch`
specify the new ip-adress. For the red bebop this is `10.42.0.102`.

In the package `multiple_bebops` there is some documentation in how we made it possible to connect to our wifi using the on/off-button

## Telnet into the Bebop
1. Enable telnet on the drone by quickly pressing the power button 4 times
2. Telnet into the drone using `$telnet <ip address>`

The `<ip address>` depends on whether you have connected to `Bebop2-xxxxx` wifi, in which case it is `192.168.42.1` or if you have made the bebop connect to the `WASP1-24` wifi as described above, in which case the addres is given above.

## Send and publish topics to the Bebop 2 drone

Publish a message once to make it takeoff

```
$ rostopic pub --once /bebop/takeoff std_msgs/Empty
```
Echo a message once to make it takeoff

```
$ rostopic echo /bebop/takeoff 
```

## Control the camera angle of the bebop

Publish a `geometry_msgs/Twist` to topic `[namespace]/camera_control`

Vertical tilting: around the y-axis
Horiontal paning: around the z-axis
angular.y  (+) upwards and (-) downwards
angular.z (+) pan left (-) pan right

```
$ rostopic pub --once /bebop/camera_control geometry_msgs/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: -60, z: 0}}"
```

# Running the Bebop

## Step 1 - start the driver

Starting the Bebop driver is sometimes easier said than done...

First, make sure that the bebop drone is connected to the wifi.
Second, run the launch file:

```
$ roslaunch bebop_driver bebop_nodelet.launch
```

or

```
$ roslaunch bebop_driver bebop_node.launch
```

To see if it works or not it is suggested to also run

```
$ rostopic echo /bebop/odom
```

If the intialization of the driver is sucessfull it should broadcast more than 32 frames of `/bebop/odom`

**If it freezes (it happens!) you have to kill the driver and restart
it. This might lead to the bebop loosing connection to the wifi... In
that case just restart the bebop, reconnect it to the wifi and try
again.**

NOTE: sometimes it can take a few tries to get it up and running...

### Specify namespace for the bebop-driver

This is done in the launchfile `bebop_autonomy/bebop_driver/launch/bebop_nodelet.launch`

## Step 2 - start `bebop action server` + `bebop controller` + `teleoperator`

This is currently easiest done through running the launch file
`coordinator/launch/no_rosplan_test_bebop.launch`

This starts the `BebopActionServer` which in turn starts the `bebop controller`.

The launchfile also starts the `coordinator` and `action_feeder` so if
you just want to run the bebop without these you can just comment them
out from the launch-file (or make a new one)

## Alt. step 2 - just start `bebop controller` + `teleoperator`

To start just the bebop-controller you can run

```
$ rosrun bebop_controller controller
```

and to start just the teleoperator you can running

```
$ rosrun bebop_controller teleoperator
```
