#Attach a Mesh as a Visual

**Link to Tutorial:** http://gazebosim.org/tutorials/?tut=attach_meshes

**Description**: Meshes can add realism to a model both visually and for sensors. The most common use case for a mesh is to create a realistic looking visual.

##To attach a Mesh as a Visual:

1. Open `model.sdf` file `.../.gazebo/models/my_robot/model.sdf` [model.sdf file](https://github.com/mperez13/ROS-Tutorials/blob/master/.gazebo/models/my_robot/model.sdf)
2. Add a mesh to the chassis visual. Find the visual w/ 'name ='visual'`.
  1. A mesh can come as a file on disk or from another model.
  2. for this example, use a mesh from the poineer2dx model
3. Look in locally cached model database to see if `pioneer2dx` model is there

  ```ls -l .../.gazebo/models/pioneer2dx/meshes/chassis.dae```
  1. If not you can make Gazebo pull the model from the [Model Database](https://bitbucket.org/osrf/gazebo_models) by spawning the `Pioneer 2DX` model at least once (under `Insert->http://gazebosim.org/models`) 
    1. Or you can manually download:
        
        ```
        cd ~/.gazebo/models
        wget -q -R *index.html*,*.tar.gz --no-parent -r -x -nH http://models.gazebosim.org/pioneer2dx/
        ```
        
4. In Gazebo, drag `My Robot` model in the world. 
5. The chassis is too big, so scale the visual.
6. Modify the visual to have a scaling factor.
7. Since the visual is little too low (along the z-axis), will need to raise it up by specifying a pose for the visual.


- At this point, our robot will look like a scaled down version of the Pioneer 2DX model through the GUI and to GPU based sensors such as camera, depth camera and GPU Lasers.
- Since `<collision>` elements were not modified, box geometry will still be used by the physics engine for collision dynamics and by CPU based ray sensors.

###Further Reading

-[Import a mesh tutorial](http://gazebosim.org/tutorials/?tut=import_mesh)

###Try later:

1. Find and download a new mesh on [3D Warehouse](https://3dwarehouse.sketchup.com/). Make sure the mesh is in the Collada (.dae) format.
2. Put the mesh in the `.../.gazebo/models/my_robot/meshes`, creating the `meshes` subdirectory if necessary
3. Use your new mesh on the robot, either as a replacement for the chassis, or as an additional `<visual>`.

Note: Materials (texture files such with extension like .png or .jpg), should be placed in `.../.gazebo/models/my_robot/materials/textures`


