#!/bin/bash

source /environment.sh

# Reduce camera fps

dt-exec rosparam set /"$VEHICLE_NAME"/camera_node/framerate 8

# Reduce resources consumption

dt-exec rosservice call /"$VEHICLE_NAME"/stop_sign_detector_node/switch False

dt-exec rosservice call /"$VEHICLE_NAME"/runtime_detector/switch False

dt-exec rosservice call /"$VEHICLE_NAME"/forward_kinematics_node/switch False

echo "Intersection navigation properly setup"

dt-launchfile-join