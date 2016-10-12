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
