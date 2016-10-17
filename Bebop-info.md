## Telnet into the Bebop 2 drone

1. Enable telnet on the drone by quickly pressing the power button 4 times
2. Log in with `$telnet 192.168.42.1`

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

# Connect to the Bebop with a custom IP

With the steps followd i "multiple_bebops" one has to go into the bebop-driver and change the launce-file.

i.e.

in /bebop_autonomy/bebop_driver/launch/bebop_nodelet.launch
specify the new ip-adress. For the red bebop this is "10.42.0.102".

