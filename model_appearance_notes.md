#Improve Velodyne Model Appearance 

Link to tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i2

- Simulated cameras that feed info to vision processing algorithms will benefit from models that appear realistic as well.
- Velodyne has a [STEP file](https://github.com/mperez13/ROS-Tutorials/blob/master/Downloads/HDL32E_Outline_Model.STEP)
- Gazebo can only use STL or Collada files, so we'll have to convert this file & then add it to our model.

1. Exported STEP file into [velodyne_base.dae](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/velodyne/meshes/velodyne_base.dae)
2. Modify dae file in [Blender](https://www.blender.org/) to correct units and center mesh on the origin
  1. Import `velodyne_base.dae` file
  
    ```
    File->Import->Collada
    ```
