# [Sensor Noise Model][1]

## Intro

Gazebo's sensors observes the world perfectly, so sensor noise is added to get a more realistic environment.  

- Gazebo can add noise to the following types of sensors:
  - Ray (e.g. lasers)
  - Camera
  - IMU
  
## Ray (laser) noise

- For ray sensors, add Gaussian noise to the range of each beam.
    - Gaussian noise is a statistical noise having a [probability density function (PDF)][11] equal to that of the normal distribution
- can set mean and standard deviation of the Gaussian distribution from which noise values will be sampled
- noise value is sampled independently for each beam
- after adding noise, resulting range is clamped to lie between the sensor's min and max ranges (inclusive)

#### To test the ray noise model:

1. Create a [model config file][2]:
    
```
  gedit ~/.gazebo/models/noisy_laser/model.config
```
    
2. Create a [model.sdf][3]
3. Insert a noisy laser and visualize it
4. Select the topic `/gazebo/default/hokuyo/link/laser/scan`
  - Laser View window that shows you the laser data.
      ![image of noisy laser data][4]
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

- For camera sensors, we model [output amplifier noise][5], which adds a Gaussian-sampled disturbance independently to each pixel
- can set mean and standard deviation of the Gaussian distribution from which noise values will be sampled.
- noise value is sampled independently for each pixel, then noise value is added independently for each color channel for that pixel.
- after adding noise, 
    - resulting color channel value: between 0.0 and 1.0 
    - floating pt. color value: unsigned integer in the image (usually between 0 and 255(using 8 bits per channel))

- noise model is implemented in a [GLSL][6] shader & requires a GPU to run.

#### To test the camera noise model:

1. Create a [model config file][7]:
  
  ```
    gedit ~/.gazebo/models/noisy_camera/model.config
  ```
2. Create a [model.sdf][8]: 
  
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
  2. Click on topic: '/gazebo/default/camera/link/camera/image' 
  
      ![image camera noise][12]

## IMU noise

- For IMU sensor, we model 2 kinds of disturbance to angular rates & linear accelerations:
  - noise
  - bias
- Angular rates & linear accelerations are considered seperately, which lead to 4 sets of parameters for the model:
  - rate noise
  - rate bias
  - accel noise
  - accel bias

- Noise:
  - is additive
  - is sampled from a Gaussian distribution
  - mean & standard deviation of the Gaussian distributions (one for rates & one for accels) can be set from which noise values will be sampled
  - noise value is sampled imdependently for each component (x, y, z) of each sample & added to that component
  
- Bias:
  - is additive
  - is sampled once at the start of the simulation
  - mean and the standard deviation of the Gaussian distributions (one for rates and one for accels) can be set from which bias values will be sampled
  - will be sampled according to the provided parameters
  - then with equal probability negated; the assumption is that the provided mean indicates the magnitude of the bias and that it's equal likely to be biased in either direction
  - after that bias is a fixed value, added to each component (x, y, z) of each sample

#### To test IMU noise model:

1. Create [model config file][9]: 
  
    ```
    gedit ~/.gazebo/models/noisy_imu/moodel.config
    ```
2. Create [model.sdf][10] file:

    ```
    gedit ~/.gazebo/models/noisy_imu/model.sdf
    ```
3. Start Gazebo and insert noisy IMU
4. Visualize the noisy IMU. Click `Window->Topic Visualization` and select `/gazebo/default/imu/link/imu/imu`. Click `Okay`.
  - You'll get a Text View window that shows you the IMU data

It can be difficult to apprecite the niose on a high-rate sensor like an IMU. Should be able to see the effect of large non-zero means inthe noise and/or bias parameters.

To adjust the noise, play w/ mean & standard deviation values in `model.sdf`.
- Units:
  - rate noise & rate bias are rad/s
  - accel noise & accel bias are m/s<sup>2</sup> 

[1]: http://gazebosim.org/tutorials?tut=sensor_noise&cat=sensors
[2]: ../.gazebo/models/noisy_laser/model.config
[3]: ../.gazebo/models/noisy_laser/model.sdf
[4]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/sensor_noise/files/Noisy_laser_visualizer.png
[5]: http://en.wikipedia.org/wiki/Image_noise#Amplifier_noise_.28Gaussian_noise.29
[6]: http://www.opengl.org/documentation/glsl/
[7]: ../.gazebo/models/noisy_camera/model.config
[8]: ../.gazebo/models/noisy_camera/model.sdf
[9]: ../.gazebo/models/noisy_imu/model.config
[10]: ../.gazebo/models/noisy_imu/model.sdf
[11]: https://en.wikipedia.org/wiki/Probability_density_function
[12]: https://bitbucket.org/osrf/gazebo_tutorials/raw/default/sensor_noise/files/Noisy_camera_visualizer.png
