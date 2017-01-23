# [ROS Overview][1]

##### Ros packages named [gazebo_ros_pkgs][3] provide wrappers around the stand-alone Gazebo for ROS integretion.

- provide necessary interfaces to simulate a robot in Gazebo using roS msgs, services & dynamic reconfigure 

##### Features of `gazebo_ros_pkgs`

- supports stand alone Gazebo
- build w/ [catkin][4]
- treats URDF and [SDF][5] as equally as possible
- reduces code duplication w/ Gazebo
- Improves out of the box support for controllers using `ros_control`

  ![overview of the gazebo_ros_pkgs inteface][6]

### Upgrade Gazebo-dependent packages from `simulator_gazebo` for use in your ROS packages:

#### Launch Files

- 

**Return to Gazebo Category: [Connect to ROS][2]**
[1]: http://gazebosim.org/tutorials?tut=ros_overview&cat=connect_ros
[2]: ../gazebo_categories/ros.md
[3]: http://ros.org/wiki/gazebo_ros_pkgs
[4]: http://www.ros.org/wiki/catkin
[5]: http://gazebosim.org/sdf.html
[6]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/ros_overview/figs/775px-Gazebo_ros_api.png
