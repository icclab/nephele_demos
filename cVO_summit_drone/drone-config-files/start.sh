#!/bin/bash

#Start a background screen name the session "drone":
screen -D -m -S drone &
PID=$!

#Start another window, name it "mavros" and run mavros startup in it
screen -S drone -X screen -t mavros
screen -S drone -p mavros -X exec ros2 launch mavros px4.launch
sleep 1s

#In another window we have the ros2 container without starting the robot base but the zenoh bridge
screen -S drone -X screen -t zenoh
screen -S drone -p zenoh -X exec zenoh-bridge-ros2dds -c /home/orin/nephele_demos/cVO_summit_drone/drone-config-files/zenoh-bridge-conf.json5
sleep 1s

#In the same container we will also start the zed
# screen -S drone -X screen -t zed
# screen -S drone -p zed -X exec ros2 launch zed_wrapper zed_camera.launch.py
# sleep 1s

# #In the same container we will also start the vo-wot environment
screen -S drone -X screen -t vo-wot
screen -S drone -p vo-wot -X exec vo-wot -t /home/orin/nephele_demos/cVO_summit_drone/drone-config-files/drone-td.json -f /home/orin/nephele_demos/cVO_summit_drone/drone-config-files/drone.yaml /home/orin/nephele_demos/cVO_summit_drone/drone-config-files/drone-mavros.py

#In the same container we will also start markerarray publisher
# screen -S drone -X screen -t marker
# screen -S drone -p vo-wot -X exec python3 /home/orin/ros2_ws/src/pc2_to_grid/pc2_to_grid/leakage.py

#In the same container we will also start the fake data
# screen -S drone -X screen -t fake
# screen -S drone -p vo-wot -X exec python3 /home/orin/ros2_ws/src/pc2_to_grid/pc2_to_grid/test.py


#Attach to screen
#screen -r drone

#Echo help
echo ""
echo "1) Stop this script (ctrl-z) and put it in background (bg) if it's not already"
echo "2) Attach to screen with: screen -r drone / Detach with ctrl-a ctrl-d"
echo "3) Kill screen with : screen -S drone -X quit"

#Wait fo screen to end
wait $PID
