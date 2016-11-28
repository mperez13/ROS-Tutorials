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

1. Create a [model config file]():
    
    ```
      gedit ~/.gazebo/models/noisy_laser/model.config
    ```

