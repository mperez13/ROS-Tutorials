#Sensor Noise Model

Link to tutorial - http://gazebosim.org/tutorials?tut=sensor_noise&cat=sensors

##Intro

- Gazebo can add noise to the following types of sensors:
  - Ray (e.g. lasers)
  - Camera
  - IMU
  
##Ray (laser) noise

- For ray sensors, we add Gaussian noise to the range of each beam.
- can set mean and standard deviation of the Gaussian distribution from which noise values will be sampled
- noise value is sampled independently for each beam
- after adding noise, resulting range is clamped to lie between the sensor's min and max ranges (inclusive)

#####To test the ray noise model:

1. Create a [model config file](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/noisy_laser/model.config):
    
    ```
      gedit ~/.gazebo/models/noisy_laser/model.config
    ```
2. Create a [model.sdf](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/noisy_laser/model.sdf)
3. Insert a noisy laser and visualize it
4. Select the topic `/gazebo/default/hokuyo/link/laser/scan`
  - Laser View window that shows you the laser data.
      ![image of noisy laser data](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/sensor_noise/files/Noisy_laser_visualizer.png)
  1. As you can see, scan is noisy. 
    1. to adjust the noise, play w/ the mean & standard deviation values in `model.sdf`, where units are meters:
    
        ```
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        ```
        These are reasonable values for Hokuyo lasers.

##Camera noise





