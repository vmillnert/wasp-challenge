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

