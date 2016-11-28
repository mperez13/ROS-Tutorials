#Sensor Noise

Link to Tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i3

**This tutorial will improve the output from the sensor through the addition of noise.**

- Every sensor has noise in the output.
- Cameras can have chromatic aberrations, sonars multi-path effects, and lasers incorrect distance readings.
- Gazebo's built in noise model applies Gaussian moise to a variety of sensors.
  - it is not perfect, but it serves as a good first-pass approximation of noise
  - generally easy to apply to data streams
  
More info on [Gazebo's sensor noise model](https://github.com/mperez13/ROS-Tutorials/blob/master/gazebo_notes/sensor_noise_model_info.md)


##Step 1: Visualize the sensor data

1. Open Gazebo and insert Velodyne sensor
2. Add a box in front of the laser beams
  1. Select Box icon in toolbar on top
3. Take a closer look at the sensor data through Gazebo's topic visualizer
  1. Press Ctrl-t to open topic selector
      Find `/gazebo/default/velodyne/top/sensor/scan` topic and select it
  2. Press Okay to open a laser visualizer
3. Notice the nice smooth lines of the output
    ![Image of sensor data](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/guided_i/files/velodyne_vis_no_noise.png)

##Step 2: Add noise to the sensor


