# Homework2

## :package: About

This package contains the developed code for the second homework of the Robotics Lab 2024/25 Course. The authors of the package are:
William Notaro, Chiara Panagrosso, Salvatore Piccolo, Roberto Rocco

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace.  If you want to only clone the content files without creating the repo folder, use
```
$ git clone https://github.com/well-iam/RL24-RepoHomework2
```
Build the two packages
```
$ colcon build
```
Source the setup files
```
$ source ~/ros2_ws/install/setup.bash
```

## :white_check_mark: Usage
Run the simulation environment through the launch file (by default the Gazebo simulation starts in the "PAUSED" state. To make it play click on the bottom left "Play" button):
```
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller"
```
where:
- 'command_interface' selects the command interface and can be chosen among: 'position', 'velocity' and 'effort'
- 'robot_controller' is the controller correspondent to the command interface and has to be accordingly chosen: 'iiwa_arm_controller', 'velocity_controller' and 'effort_controller'
Set "command_interface" to "effort" and "robot_controller" to "effort_controller" for starting our effort controller. 

To run the controller, use the following code:
```
$ ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:="effort_joint" -p traj_sel:=1
```
where:
- 'cmd_interface' selects the command interface and can be chosen among: 'position', 'velocity', 'effort_joint' and 'effort_operational'
- 'traj_sel' selects the desired trajectory and can be chosen among:
    - '0' for a linear segment with a cubic polynomial time law
    - '1' for a linear segment with a trapezoidal velocity profile
    - '2' for a circular arc with a cubic polynomial time law
    - '3' for a circular arc with a trapezoidal velocity profile

## :warning: Warning
Due to the nature of the project, you have to be quick when running the two parts. You have 5 seconds (a default timeout value difficult to change) to start the Gazebo program, connect the controller and play the simulation. You may want to first run the controller - which will be but on hold - and then the launch file.
