#!/bin/bash

mkdir -p logs;
echo Launch the world
roslaunch world simulator.launch >logs/simulator.log 2>&1 &
sleep 5;
echo Bring in the turtle
export ROBOT_INITIAL_POSE="-x -0.5 -y -0.5";
roslaunch wasp_g1_start start_turtlebot_sim.launch >logs/turtle.log 2>&1 &
sleep 5;
echo Add drone
roslaunch world ardrone_sim.launch >logs/drone.log 2>&1;

