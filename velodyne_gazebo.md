#Create SDF model of the [Velodyne HDL-32 LiDAR](http://velodynelidar.com/hdl-32e.html) sensor

Link to tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i1

Based on the Velodyne documentation, wll create a sensor that has:

1. a base cylinder & top cylinder, where the top cylinder spins, and
2. a set of laser rays oriented in the vertical fan

##Step 1: Create a basic SDF model

1. Create a new world file

  ```
  cd
  gedit velodyne.world
  ```
2. Populate world file w/ a ground plane and light. [velodyne.world](https://github.com/mperez13/ROS-Tutorials/blob/master/velodyne.world)
3. Add the basics of the Velodyne LiDAR to the SDF world file.
  - Use the Velodyne sensor dimensions to construct a base cylinder & a top cylinder.
  - [Velodyne 2D drawing](http://velodynelidar.com/lidar/hdldownloads/86-0106%20REV%20A%20OUTLINE%20DRAWING%20HDL-32E.pdf)
4. Run the Velodyne world

  ```
  cd
  gazebo velodyne.world -u
  ```
5. By default:
  - \<visual> elements: define how the model looks
  - 


