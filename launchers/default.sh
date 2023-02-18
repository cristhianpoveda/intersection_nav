#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launching app
dt-exec roslaunch intersection_driving intersection_driving.launch veh:="$VEHICLE_NAME"


# wait for app to end
dt-launchfile-join
