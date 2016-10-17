# Telnet into the Bebop 2 drone

1. Enable telnet on the drone by quickly pressing the power button 4 times
2. Log in with `$telnet 192.168.42.1`

# Send and publish topics to the Bebop 2 drone

Publish a message once to make it takeoff

```
$ rostopic pub --once /bebop/takeoff std_msgs/Empty
```
Echo a message once to make it takeoff

```
$ rostopic echo /bebop/takeoff 
```


# Connect to the Bebop with a custom IP

With the steps followd i "multiple_bebops" one has to go into the bebop-driver and change the launce-file.

i.e.

in /bebop_autonomy/bebop_driver/launch/bebop_nodelet.launch
specify the new ip-adress. For the red bebop this is "10.42.0.102".