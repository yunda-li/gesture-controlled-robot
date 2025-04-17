#!/bin/bash
source /environment.sh


# initialize launch file
dt-launchfile-init


# launch publisher
rosrun testing pose_detect_test.py


# wait for app to end
dt-launchfile-join
