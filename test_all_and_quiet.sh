#!/bin/bash

if [[ ! -f ./build/u_turn_planner_test ]]; then
    echo "please first build this project and make sure path:build/u_turn_planner_test is exist"
    exit 1
fi

echo ${1}
for test_file in ./test_data/*.txt
do
    echo "use data: ${test_file}"
    ./build/u_turn_planner_test 1.0 3 40 -40 40 1.0 ${test_file}
done
