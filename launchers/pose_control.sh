#!/bin/bash
source /environment.sh


# initialize launch file
dt-launchfile-init


# launch publisher
rosrun testing pose_control(REF).py


# wait for app to end
dt-launchfile-join