# AutoDrone
Autonomous navigation in an unknown environment using PX4 and LDRobot's LD06 lidar


This code is part of the scientific paper "Engineering, Construction, and Programming of a Fully Autonomous Drone".

# Introduction
The autonomous navigation is handled by the exploration_mode and the waypoint_mode functions (detailed description in found in the paper). The algorithm will search for a 3 metres wide path with an acute angle to the line connecting the drone and the destination. If no path is found, the drone will attempt to fly over the obstacle.

# compile and run
Compile using makefile
``` bash
cd ~/AutoDrone/
make
```

run
``` bash
cd ~/AutoDrone/
./main
```
After entering the target coordinates the drone will takeoff, fly to destination (exploration_mode), return to start (waypoint_mode), and land.


# Dependencies
Mavlink: https://github.com/mavlink/c_library_v2/tree/d8fcf0a694dc11b3f83b89a0970e3d8c4e48d418

# Paper
to be added

# Credits
The projects c_uart_interface_example by mavlink and ldlidar_stl_sdk by ldrobotSensorTeam have helped the development of this project tremendously.

https://github.com/ldrobotSensorTeam/ldlidar_stl_sdk

https://github.com/mavlink/c_uart_interface_example

# Disclaimer
USE WITH CAUTION AND IN COMPLIANCE WITH LOCAL LAWS!
