# [Connect to ROS][1]

### ROS is a robot control framework.  These tutorials describe how to interface Gazebo and ROS

|Title|Description|Notes|
|----|----|----|
|[ROS Overview][2]|Gives basic description of `gazebo_ros_pkgs`. Guidelines for upgrading Gazebo-dependend packages for use in ROS packages.||
|[Versions to Use][3]|Gives description of different versions of ROS and Gazebo.|The one we used is Kinetic.|
|[Installing gazebo_ros_pkgs][4]|Set of ROS packages for interfacing w/ Gazebo are contained w/in a new meta package (catkin's version of stacks) named gazebo_ros_pkgs. `rosrun` commands for starting Gazebo.||
|[Using roslaunch][5]|using rosrun and roslaunch. This includes storing your URDF files in ROS packages and keeping your various resource paths relative to your ROS workspace.||
|[URDF in Gazebo][6]|Explains the necessary steps to successfully use your URDF-based robot in Gazebo, saving you from having to create a separate SDF file from scratch and duplicating description formats.|*NOT DONE*|
|[Gazebo plugins in ROS][7]|explain both how to setup preexisting plugins and how to create your own custom plugins that can work with ROS.|*NOT DONE*|
|[ROS Control][8]|Setup simulated controllers to actuate the joints of your robot. This will allow us to provide the correct ROS interfaces|*NOT DONE*|
|[ROS Communication][9]|Demonstrate some of the utilities for manipulating the simulation world and objects. The complete list of ROS messages and services for gazebo can be found here also.|*NOT DONE*|
|[ROS Plugin][10]|Creating a very basic Gazebo plugin that is ROS-aware.|*NOT DONE*|
|[Advanced ROS Integration][11]||*NOT DONE*|

**Return to [Categories][12]**

[1]: http://gazebosim.org/tutorials?cat=connect_ros
[2]: ../gazebo_notes/ros_overview.md
[3]: http://gazebosim.org/tutorials?tut=ros_wrapper_versions&cat=connect_ros
[4]: ../gazebo_notes/install_pkgs.md
[5]: ../gazebo_notes/roslaunch.md
[6]: ../gazebo_notes/urdf.md
[7]: ../gazebo_notes/plugins_in_ros.md
[8]: ../gazebo_notes/ros_control.md
[9]: ../gazebo_notes/ros_communication.md
[10]: ../gazebo_notes/ros_plugin.md
[11]: ../gazebo_notes/advance_ros_integration.md
[12]: ../gazebo_notes.md
