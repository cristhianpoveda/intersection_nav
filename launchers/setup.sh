#!/bin/bash

source /environment.sh

# Reduce camera fps

dt-exec rosparam set /"$VEHICLE_NAME"/camera_node/framerate 8

# Set maximum velocities

dt-exec rosparam set /"$VEHICLE_NAME"/kinematics_node/v_max 0.2

dt-exec rosparam set /"$VEHICLE_NAME"/kinematics_node/omega_max 8.0

dt-exec rosparam set /"$VEHICLE_NAME"/kinematics_node/limit 1.0

# Reduce resources consumption

dt-exec rosservice call /"$VEHICLE_NAME"/stop_sign_detector_node/switch False

dt-exec rosservice call /"$VEHICLE_NAME"/runtime_detector/switch False

dt-exec rosservice call /"$VEHICLE_NAME"/forward_kinematics_node/switch False

echo "Setting up intersection navigation!"

dt-launchfile-join