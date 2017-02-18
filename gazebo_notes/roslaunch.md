# [Using roslaunch to start Gazebo, world files, and URDF models][1]

## Using `roslaunch` to Open World Models

- [roslaunch][3] tool is the standard method for starting ROS nodes & bringing up robots in ROS.
- Start empty Gazebo world

  ```
  roslaunch gazebo_ros empty_world.launch
  ```

### `roslaunch` Arguments

||description|default|
|-----|-----|-----|
|**paused**|start Gazebo in a paused state|false|
|**use_sim_time**|tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic /clock|true|
|**gui**|launch user interface window of Gazebo|true|
|**headless**|disable any function calls to simulator redenring (Ogre) components. does not work w/ gui:=true|false|
|**debug**|start gzserver in debug mode using gdb|false|

- example:
  
  ```
  roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false headless:=false debug:=true
  ```

### Launching Other Demo Worlds

- other demo worlds are already included in `gazebo_ros` package
  
  ```
  roslaunch gazebo_ros willowgarage_world.launch
  roslaunch gazebo_ros mud_world.launch
  roslaunch gazebo_ros shapes_world.launch
  roslaunch gazebo_ros rubble_world.launch
  ```
  
- example: [samplebot.launch][4]

### World Files

- example: [samplebot.world][5]

#### Finding World File On Your Computer

- world files are found w/in `/worlds` directory of your Gazebo resource path
- to find location:
  
  ```
  env | grep GAZEBO_RESOURCE_PATH
  ```

- you can set your path by adding this to your `.bashrc` file
  
  ```
  export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-7/worlds:$GAZEBO_RESOURCE_PATH
  ```

## Creating your own Gazebo ROS Package

- per ROS standards,
  - robot's model and description is located in a package named `/MYROBOT_description` 
  - all the world & launch files used in Gazebo are located in a package named `/MYROBOT_gazebo`
  > replace `MYROBOT` w/ the name of your bot in lower case  
- sample package hierarchy
  
  ```
  ../catkin_ws/src
    /MYROBOT_description
        package.xml
        CMakeLists.txt
        /urdf
            MYROBOT.urdf
        /meshes
            mesh1.dae
            mesh2.dae
            ...
        /materials
        /cad
    /MYROBOT_gazebo
        /launch
            MYROBOT.launch
        /worlds
            MYROBOT.world
        /models
            world_object1.dae
            world_object2.stl
            world_object3.urdf
        /materials
        /plugins
  ```

- command `catkin_create_pkg` is used for creating new packages

### Create Custom World File

- Create ROS package w/ the convention MYROBOT_gazebo
  - w/in this package, create a [MYROBOT.launch][4]
 
- also create `worlds` folder & create [MYROBOT.world][5] file:

- Launch custom world 
  - I created a script file to launch file
    - launch file in workspace folder (`~/ROS-Tutorials/controlros_ws`) by running it like:
    
      ```
      ./run_gazebo.sh
      ```

## Using `roslaunch` to Spawn URDF Robots

### ROS Service Call Spawn Method

- this method keep your robot's ROS packages more portable between computers & repository check outs
- it allows you to keep your robot's location relative to a ROS package path, but also requires you to make a ROS service call using a small (python) script called `spawn_model`
- `spawn_model` makes service call request to the `gazebo_ros` ROS node to add custom URDF into Gazebo
  - is located w/in `gazebo_ros` package
  - used the following way
    
     ```
     rosrun gazebo_ros spawn_model -file `rospack find MYROBOT_description`/urdf/MYROBOT.urdf -urdf -x 0 -y 0 -z 1 -model MYROBOT
     ```

  - to view available argument for `spawn_model`
    
    ```
    rosrun gazebo_ros spawn_model -h
    ```
  
#### URDF Example w/ Baxter

- download baxter_description package from Rethink Robotic's [baxter_common][6] repo, if you do not have a URDF to test

  ```
  git clone https://github.com/RethinkRobotics/baxter_common.git
  ```

- should now have a URDF file named `baxter.urdf` that you can run:
  
  ```
  rosrun gazebo_ros spawn_model -file `rospack find baxter_description`/urdf/baxter.urdf -urdf -z 1 -model baxter
  ```

- to integrate into a ROS launch file, reopen file `MYROBOT_gazebo/launch/YOUROBOT.launch` and add the following before the `</launch>` tag:
  
  ```
  <!-- Spawn a robot into Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find baxter_description)/urdf/baxter.urdf -urdf -z 1 -model   baxter" />
  ```
  
  > Launching this file should give you the same result as when using `rosrun`

#### Using [XACRO][8] when URDF is not in XML format

- Add this to the launch file:
  
  ```
  <!-- Convert an xacro and put on parameter server -->
  <param name="robot_description" command="$(find xacro)/xacro.py $(find pr2_description)/robots/pr2.urdf.xacro" />

  <!-- Spawn a robot into Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model pr2" />
  ```



### Model Database Method

- this method allows you to include your robot w/in the `.world` file
- seems more convenient but it requires you to add your robot to the Gazebo model database by setting an evironment variable
  - environment variable is required because of the separation of ROS dependencies from Gazebo
  - URDF package paths cannot be used directly inside `.world` files because Gazebo does not have a notion og ROS packages

- to accomplish this method
  - make a new model database containing your single robot
    - refer back to the Gazebo Model Database
  - the ROS workspace file hierarchy will now look like:
  
    ```
    ../catkin_ws/src
    /MYROBOT_description
        package.xml
        CMakeLists.txt
        model.config
        /urdf
            MYROBOT.urdf
        /meshes
            mesh1.dae
            mesh2.dae
            ...
        /materials
        /plugins
        /cad
    ```

- this hierarchy is specially adapted for use as a Gazebo model database by means of the following:
  - **/home/user/catkin_workspace/src** - location of a Gazebo Model Database
  - **/MYROBOT_description** - directory treaed as a single Gazebo model folder 
  - **model.config** - required configuration file for Gazebo to find this model in its database 
  - **MYROBOT.urdf** - description file, also used by Rviz, MoveIt!, etc. 
  - **/meshes** - location for .stl or .dae files 

#### [model.config][7]

- this file contains meta info about the model
- unlike for SDF's, no version is required for the tag when it is used for URDFs

#### Environment Variable

- Add environment variable to your .bashrc file, which tells Gazebo where to look for model database
- Check if you have a `GAZEBO_MODEL_PATH` defined
  - if you already have one, append to it using a semi-colon
  - if not add the new export
- path should look like:
  
  ```  
  export GAZEBO_MODEL_PATH=/home/user/catkin_ws/src/
  ```

#### Viewing in Gazebo - Manually

- start up Gazebo
  
  ```
  gazebo
  ```

- Insert models 
  - Click "Insert" tag
  - find database corresponding to your robot, open a sub folder, click name of your robot and insert it

#### Viewing in Gazebo - `roslaunch` w/ the Model Database

- Advantage of model database method is you can include MYROBOT to your world files, w/out using ROS package path
- w/in `MYROBOT_description/launch` folder, edit [MYROBOT.world][5]:
- should now be able to launch custom world file:
  
  ```
  roslaunch MYROBOT_gazebo MYROBOT.launch
  ```

- disadvantage of this method is that `MYROBOT_description` & `MYROBOT_gazebo` are not easily portable between computers
  - you first have to set `GAZEBO_MODEL_PATH` on any new system before being able to use these ROS packages
  
## Exporting model paths from a package.xml

- useful info would be the format for exporting model paths from a package.xml
  
  ```
  <export>
    <gazebo_ros gazebo_model_path="${prefix}/models"/>
    <gazebo_ros gazebo_media_path="${prefix}/models"/>
  </export>
  ```

**Return to Gazebo Category: [Connect to ROS][2]**

[1]: http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros
[2]: ../gazebo_categories/ros.md
[3]: http://www.ros.org/wiki/roslaunch
[4]: ../controlros_ws/src/mybot_gazebo/launch/samplebot.launch
[5]: ../controlros_ws/src/mybot_gazebo/worlds/samplebot.world
[6]: https://github.com/RethinkRobotics/baxter_common
[7]: ../controlros_ws/src/mybot_description/model.config
[8]: http://ros.org/wiki/xacro
