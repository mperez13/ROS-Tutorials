#Sensor Noise

Link to Tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i3

**This tutorial will improve the output from the sensor through the addition of noise.**

- Every sensor has noise in the output.
- Cameras can have chromatic aberrations, sonars multi-path effects, and lasers incorrect distance readings.
- Gazebo's built in noise model applies Gaussian moise to a variety of sensors.
  - it is not perfect, but it serves as a good first-pass approximation of noise
  - generally easy to apply to data streams
  
More info on [Gazebo's sensor noise model]()
