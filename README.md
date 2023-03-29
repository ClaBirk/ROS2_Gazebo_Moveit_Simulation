# Startup

## Installation of required packages:

### ROS2 Humble
`sudo apt install ros-humble-desktop-full`
### Gazebo
`curl -sSL http://get.gazebosim.org | sh`
`sudo apt-get install ros-humble-gazebo-ros`
`sudo apt-get install ros-humble-ros2-control`
`sudo apt-get install ros-humble-gazebo-ros2-control`
`sudo apt install ros-humble-xacro`
`sudo apt-get install ros-humble-joint-state-publisher-gui*`
`sudo apt-get install ros-humble-joint-state-broadcaster`
`sudo apt-get install ros-humble-joint-trajectory-controller`
`sudo apt-get install ros-humble-controller-manager`
`sudo apt-get install ros-humble-ros2-controllers`
`sudo apt-get install ros-humble-simulation`

### Moveit
`sudo apt-get install ros-humble-moveit`

## Launch 
`ros2 launch ros_sim gazebo_moveit.launch.py`

The specified lauch will start the Gazebo and Moveit Simulation.
In addition the packages contain several other launch files to launch different parts of the whole simulation (discribed above).

To start the Robot - will execute a linear tool path required for milling application:
`ros2 run ros_milling_cpp ros_milling_cpp_full`

# Want to implement control for your own Robot?
## Required steps:
1. create a own moveit setup with the `ros2 launch moveit_setup_assistant setup_assistant.launch.py` - Remember it must be named: XYZ_moveit_config. (You might need to change the joint_limits.yaml to "has_acceleration_limits: true max_acceleration: *desired value*"). The here described config can be an example. It was generated with the ros_sim/urdf/kr10_full_moveit_gazebo.xacro
3. Change the launch file in ros_sim package accordingly
4. ros_milling_cpp - change the Group name to start planning

__Please contact me if there are any issues.__
__Link to this repository if u made any improvements.__
__Dont use this package for any payed tutorial about ROS2 Humble - it is meant to be easy to use without any instruction.__
