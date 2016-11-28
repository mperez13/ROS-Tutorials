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

- For camera sensors, we model [output amplifier noise](http://en.wikipedia.org/wiki/Image_noise#Amplifier_noise_.28Gaussian_noise.29), which adds a Gaussian-sampled disturbance independently to each pixel
- can set mean and standard deviation of the Gaussian distribution from which noise values will be sampled.
- noise value is sampled independently for each pixel, then noise value is added independently for each color channel for that pixel.
- after adding noise, resulting color channel value is clamped to lie between 0.0 and 1.0 (floating pt. color value end up as unsigned integer in the image, usually between 0 and 255(using 8 bits per channel))

- noise model is implemented in a [GLSL](http://www.opengl.org/documentation/glsl/) shader & requires a GPU to run.

#####To test the camera noise model:

1. Create a [model config file](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/noisy_camera/model.config):
  
  ```
    gedit ~/.gazebo/models/noisy_camera/model.config
  ```
2. Create a [model.sdf](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/noisy_camera/model.sdf):
  
  ```
    gedit ~/.gazebo/models/noisy_camera/model.sdf
  ```
3. Insert a noisy camera and visualize it.
  1. To adjust the noise, simply play w/ the mean & standard deviation values in the `model.sdf`
  
  ```
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.007</stddev>
    </noise>
  ```

##IMU noise

NEED TO FINISH TAKING NOTES







