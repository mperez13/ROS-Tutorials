#Improve Velodyne Model Appearance 

Link to tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i2

- Simulated cameras that feed info to vision processing algorithms will benefit from models that appear realistic as well.
- Velodyne has a [STEP file](https://github.com/mperez13/ROS-Tutorials/blob/master/Downloads/HDL32E_Outline_Model.STEP)
- Gazebo can only use STL or Collada files, so we'll have to convert this file & then add it to our model.

##Step 1: Mesh Acquisition

1. Exported STEP file into [velodyne_base.dae](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/velodyne/meshes/velodyne_base.dae)
2. Modify dae file in [Blender](https://www.blender.org/) to correct units and center mesh on the origin
  1. Open Blender using Command Line
  
    ```
    blender
    ```
  2. Import `velodyne_base.dae` file
  
    ```
    File->Import->Collada
    ```
  3. Units are in millimeters & Gazebo requires meters.
  4. On right tab, click plus sign, Under `Dimesions` section, divide x,y,z components by 1000.
  5. In same tab, rotate model by 90 degrees around the X-axis
3. Export the mesh as a Collada file 
  - [velodyne_base.dae](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/velodyne/meshes/velodyne_base.dae)
4. Export the mesh as a Collada file 
  - [velodyne_top.dae](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/velodyne/meshes/velodyne_top.dae)

##Step 2: Add meshes to SDF

1. Add files velodyne_base.dae & velodyne_top.dae to:

  ```
  ~/.gazebo/models/velodyne_hdl132/meshes
  ```
2. Modify model's SDF to use `velodyne_top` mesh - 
  
