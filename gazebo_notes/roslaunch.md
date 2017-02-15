# [Using roslaunch to start Gazebo, world files, and URDF models][1]

## Using `roslaunch` to Open World Models

- [roslaunch][3] tool is the standard method for starting ROS nodes & bringing up robots in ROS.
- Start empty Gazebo world

  ```
  roslaunch gazebo_ros empty_world.launch
  ```

### `roslaunch` Arguments

||description|default|
|-----|-----|-----|
|**paused**|start Gazebo in a paused state|false|
|**use_sim_time**|tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic /clock|true|
|**gui**|launch user interface window of Gazebo|true|
|**headless**|disable any function calls to simulator redenring (Ogre) components. does not work w/ gui:=true|false|
|**debug**|start gzserver in debug mode using gdb|false|

- example:
  
  ```
  roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false headless:=false debug:=true
  ```

### Launching Other Demo Worlds

- other demo worlds are already included in `gazebo_ros` package
  
  ```
  roslaunch gazebo_ros willowgarage_world.launch
  roslaunch gazebo_ros mud_world.launch
  roslaunch gazebo_ros shapes_world.launch
  roslaunch gazebo_ros rubble_world.launch
  ```
  
- example: [mud_world.launch][4]

## Creating your own Gazebo ROS Package


## Using `roslaunch` to Spawn URDF Robots


## Exporting model paths from a package.xml



**Return to Gazebo Category: [Connect to ROS][2]**

[1]: http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros
[2]: ../gazebo_categories/ros.md
[3]: http://www.ros.org/wiki/roslaunch
[4]: ../
