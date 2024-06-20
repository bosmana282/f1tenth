## Drive the car
In order to drive the f1tenth car, two terminals have to run.

The first one is used to launch the LiDAR, the stereo camera and the ESC:
```
source install/setup.bash
sudo ip link set multicast on lo
ros2 launch f1tenth_stack bringup_launch.py
```

The second one is used to run the selected motion control and/or obstalce avoidance algorithm:
```
source install/setup.bash
ros2 run obstacle_avoidance obstacle_avoidance
```

## Implemented algorithms
From a practical point of view, all algorithms that were implemented can be found on Github in the f1tenth src-folder. With this src-folder, the build- and install-folder can be build with the command \textit{colcon build} in the context of ROS2. In the source folder, seven folders are present:

1. `f1tenth_system`, the folder that defines and allows to launch all components on the car. It was taken from the official Github page of the f1tenth-foundation and was adapted to integrate the LiDAR, ESC and stereo camera in the setup.
2. `realsense-ros`, which was taken from the official Github page of Intel. It allows the user to integrate the functionalities of the Intel Realsense D435i stereo camera into the f1tenth_system file.
3. `sllidar_ros2`, which was taken from the official Github page of RPLIDAR. It allows the user to integrate the functionalities of the RPLIDAR S1 LiDAR into the f1tenth_system file.
4. `feedback_control`, a folder that implements the theoretical aspects of feedback control on the f1tenth. This motion control algorithm was created from scratch.
5. `pure_pursuit`, a file that implements the theoretical aspects of pure pursuit on the f1tenth. This motion control algorithm was created from scratch.
6. `obstacle_avoidance`, a folder that exists of two main python-scripts: waypoint_navigation and obstacle_avoidance. The former is a generalization of the code in feedback_control and pure_pursuit. It allows the user to select the preferred motion control algorithm to steer the car along a predefined set of waypoints. The latter is a general code that allows the user to select his preferred motion control and obstacle avoidance algorithm to guide the car along a set of predefined waypoints. 
7. `safety_node`, which implements a simple AEB safety system for the f1tenth car. 
