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

## Guidelines to Upgrade Gazebo-dependent packages from `simulator_gazebo` for use in your ROS packages:

#### Launch Files 

- w/in roslaunch files, `pkg="gazebo"` needs to be now renamed to `pkg="gazebo_ros`
- `gazebo_worlds` package has been removed
  - most world files were rarely used & not maintained w/ chnages in SDF XML formats. So all worlds have been centralized w/in Gazebo project itself, including `empty.world`
- best way to use launch file is to inherit/include the master `empty_world` launch file located in `gazebo_ros` package

#### CMakeLists.txt

ROS-wrapped version of Gazebo was removed in favor of the system install of Gazebo, so you may require configuration of your CMake file.

Example [CMakeLists.txt][7] file

#### package.xml



#### Running Gazebo



**Return to Gazebo Category: [Connect to ROS][2]**

[1]: http://gazebosim.org/tutorials?tut=ros_overview&cat=connect_ros
[2]: ../gazebo_categories/ros.md
[3]: http://ros.org/wiki/gazebo_ros_pkgs
[4]: http://www.ros.org/wiki/catkin
[5]: http://gazebosim.org/sdf.html
[6]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/ros_overview/figs/775px-Gazebo_ros_api.png
[7]: ../ros_overview/CMakeLists.txt
