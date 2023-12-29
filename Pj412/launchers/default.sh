#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec roslaunch parking_node parking_node.launch veh:="$VEHICLE_NAME"
dt-exec roslaunch apriltag_node apriltag_node.launch veh:="$VEHICLE_NAME"
sleep 6
dt-exec roslaunch lane_following lane_following.launch veh:="$VEHICLE_NAME"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
