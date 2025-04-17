#!/bin/bash
source /environment.sh


# initialize launch file
dt-launchfile-init


# launch publisher
rosrun testing pose_processor.py &

rosrun testing motor_command.py &

# wait

# wait for app to end
dt-launchfile-join
