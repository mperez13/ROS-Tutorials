# Customize Simulation - Velodyne LiDAR Sensor

|Title|Description|Notes|
|-----|-----|-----|
|[Create SDF Model of the Velodyne Sensor][8]| Base on the Velodyne documentation, create basic .sdf file to model the sensor||
|[Model Appearance][9]| Improve the Velodyne model's appearance by creating meshes||
|[Sensor Noise][7]| Add sensor noise to Velodyne model in order to improve the output from the sensor||
|[Control Plugin][10]| Create a plugin in a new directory. The contents of this directory will include the plugin source code, and CMake build script||
|[Connect to ROS][11]| Use the plugin to connect to ROS. Plugin will be loaded as usual and will listen on ROS topic for incoming float messages, that will be used to set Velodyne's rotational speed|Connect ROS w/ Gazebo through the command line|

[7]: ../gazebo_notes/sensor_noise.md
[8]: ../gazebo_notes/velodyne_gazebo.md
[9]: ../gazebo_notes/model_appearance_notes.md
[10]: ../gazebo_notes/control_plugin.md
[11]: ../gazebo_notes/connect_to_ROS.md
