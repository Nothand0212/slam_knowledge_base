#!/bin/bash

# check roscore running or not
if ! rostopic list > /dev/null 2>&1; then
    echo "roscore no running, start roscore..."
    roscore > /tmp/roscore.log 2>&1 &
    ROSCORE_PID=$!
    sleep 3  # waitting for roscore
else
    echo "roscore is running"
fi

echo "start rviz..., and load config"
rviz -d config/rviz/mapping.rviz

