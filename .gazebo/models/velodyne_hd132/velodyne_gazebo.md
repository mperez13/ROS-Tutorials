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
  - `\<visual>` elements: define how the model looks
  - `\<collision>` elements: define how the model will behave when colliding w/ other models
6. To debug the `\<collision>` elements `Right-click` on a model and choose `View->Collisions`

##Step 2: Add Inertia

- The physics engine uses inertia info to calculate how a model will behave when forces act upon it.

1. Start by visualizing the current inertia values. 
  - click on model and select `View->Inertia`
2. Add inertia to a link by specifying mass and inertia matrix
  - mass = 1.3kg 
  - moment of inertia can be computed by using [equations](https://en.wikipedia.org/wiki/List_of_moments_of_inertia)

##Step 3:Add the joint

- Joints define constraints between links.
- most common type of joint is `revolute`
  - revolute: single rotational degree of freedom between two links
- [List of joints](http://sdformat.org/spec?ver=1.6&elem=joint#joint_type)
- to visualize joints, click on model & choose `View->Joints`   
  - can make model transparent to see joint visualization (`View->Transparent`)

1. Add a joint to the Velodyne model
  1. joint will be revolute since the top link will spin relative to the base link
2. Run SDF world, `View->Joints` & `View->Transparent`
3. Verify joint rotates using Joint Command graphical tool.
  1. Drag right panel & select Velodyne model
4. Use `Force` tab to apply 0.001 to the joint. (should see the visualized joint start to spin around the model's Z-axis)

##Step 4: Add the sensor

- `ray` sensor in Gazebo consists of one or more beams that generate distance, and potentially intesity data
  - consists of:
    - `\<scan>` element defines the layout & number of beams 
      - within `\scan`:
        - `\<horizontal>` component defines rays that fan out in a horizontal plane
        - `\<vertical>` component defines rays that fan out in a vertical plane
    - `\<range>` element defines properites of an individual beam

- Velodyne sensor requires vertical rays, that then rotate
  - we will simulated this as rotated horizontal fan
- Velodyne specification indicates that the HDL-32 has 32 rays with a vertical field of view between +10.67 and -30.67 degrees

1. Add ray sensor to the top link 
2. Add `\<ray>` element, which defines `\<scan>` and `\<range>` elements.
3. Start up simulation and should see the 32 sensor beams

  
