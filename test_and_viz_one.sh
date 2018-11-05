#!/bin/bash

if [[ ! -f ./build/u_turn_planner_test ]]; then
    echo "please first build this project and make sure path:build/u_turn_planner_test is exist"
    exit 1
fi

echo ${1}
./build/u_turn_planner_test 1.0 3 40 -40 40 1.0 ${1}
if [[ ${?} != 0 ]]; then
	exit 1
fi

./viz_traj.py ${1}
