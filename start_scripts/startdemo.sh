#!/bin/bash
declare -a PIDS=()

test "x$DELAY" != "x" || DELAY=3;

mkdir -p logs;
echo Launch drone action server
roslaunch wasp_g1_start bebop_action.launch >logs/bebop_action.log 2>&1 &
PIDS+=($!);
sleep ${DELAY};


echo Start drone detector
roslaunch wasp_g1_start start_drone_tag_detector.launch >logs/start_drone_tag_detector.log 2>&1 &
PIDS+=($!);
sleep ${DELAY};

echo Start planner
export PLANNER_DOMAIN="durative_domain.pddl";
export PLANNER_WORLD="lth_robotlab.yaml";
roslaunch wasp_g1_start planner.launch >logs/planner.log &
PIDS+=($!);
sleep ${DELAY};

read -p "Press enter to exit";

echo "Killing of nodes";
kill ${PIDS[@]};
echo "Done";

