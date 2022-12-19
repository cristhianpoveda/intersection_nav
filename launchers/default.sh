#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#dt-exec roslaunch navigation forward_kinematics_node.launch veh:="$VEHICLE_NAME"
#dt-exec roslaunch navigation localization_node.launch
#dt-exec roslaunch navigation translator_node.launch veh:="$VEHICLE_NAME"
#dt-exec roslaunch navigation stop_sign_detector_node.launch veh:="$VEHICLE_NAME"
dt-exec roslaunch intersection_driving intersection_driving.launch veh:="$VEHICLE_NAME"
#dt-exec roslaunch detection object_detection_node.launch veh:="$VEHICLE_NAME"
#dt-exec roslaunch decision_making decision_making_node.launch veh:="$VEHICLE_NAME"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
