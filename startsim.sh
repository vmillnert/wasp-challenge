#!/bin/bash

mkdir -p logs;
echo Launch the world
roslaunch world simulator.launch >logs/simulator.log 2>&1 &
sleep 10;
echo Bring in the turtle
roslaunch wasp_g1_start start_turtlebot_sim.launch >logs/turtle.log 2>&1 &
sleep 10;
echo Add drone
roslaunch world ardrone_sim.launch >logs/drone.log 2>&1;

