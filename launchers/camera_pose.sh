#!/bin/bash
source /environment.sh


# initialize launch file
dt-launchfile-init


# launch publisher
rosrun testing camera_pose.py


# wait for app to end
dt-launchfile-join
