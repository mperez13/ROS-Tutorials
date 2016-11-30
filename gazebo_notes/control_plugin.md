#Control Plugin 

Link to tutorial - http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

- Plugin 
  - is a C++ library that is loaded by Gazebo at runtime
  - has access to Gazebo's API, which allows a plugin to perform tasks inlcuding moving objects, adding/removing objects & accessing sensor data
  - [Tutorials on plugins](http://gazebosim.org/tutorials?cat=write_plugin)

##Write a plugin

**Overview**: will craete the plugin in a new directory.  The contents of this directory will include the plugin source code, and CMake build script.

####Step 1: Create a [velodyne_plugin](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin) workspace

####Step 2: Create the plugin source file

1. Create [velodyne_plugin.cc](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin/velodyne_plugin.cc)

####Step 3: Create CMake build script

1. Create [CMakeLists.txt](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin/CMakeLists.txt) 

####Step 4: Attach the plugin to the Velodyne sensor

- use SDF's `\<inlcude>` capability to test out plugin w/out touching the main Velodyne SDF file

1. Create a new world file inside your workspace: [velodyne.world](https://github.com/mperez13/ROS-Tutorials/tree/master/velodyne_plugin/velodyne.world)

####Step 5: Build and test

1. W/in your workspace, create build directory and compile the plugin:

    ```
    mkdir build
    cd build
    cmake ..
    make
    ```
    - had an error when I ran `make`:
        ![Image of plugin error](https://github.com/mperez13/ROS-Tutorials/blob/master/gazebo_notes/images/plugin_error.png)
        
        - Fixed error by adding `#include <gazebo/physics/Model.hh>` to velodyne_plugin.cc
2. Run the world. Make sure to run gazebo within the `build` directory so Gazebo can find the plugin library.

    ```
    cd ~/velodyne_plugin/build
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/velodyne_plugin/build
    gazebo ../velodyne.world
    ```
3. Check your terminal, it should say:
    
    ```
    The velodyne plugin is attached to model[my_velodyne]
    ```
##Move the Velodyne

- Next add code that controls the Velodyne's joint
- We will use simple PID controller to control the velocity of the Velodyne's joint

1. Modify `Load` function in velodyne_plugin.ccand then
    
    ```
    cd build
    make
    gazebo ../velodyne.world
    ```
    The Velodyne should be spinning.
    
    **[Got an error; just continue, the rest of this tutorial will fix this problem]**

##Plugin Configuration

- In this section, we'll modify plugin to read a custom SDF parameter that is the target velocity of the Velodyne

1. Add a new element as a child of the `<plugin>` in [velodyne.world](https://github.com/mperez13/ROS-Tutorials/blob/master/velodyne_plugin/velodyne.world)
  - new element can be anything as long as it is a valid XML
  - our plugin will have access to this value in the `Load` function

  
    
    
    
    
    
    
    

  
